#include <Arduino.h>
#include "targets.h"
#include "utils.h"
#include "common.h"
#include "LowPassFilter.h"
#include "LoRaRadioLib.h"
#include "CRSF.h"
#include "FHSS.h"
// #include "Debug.h"
#include "rx_LinkQuality.h"
#include "errata.h"
#include "OTA.h"
#include "msp.h"
#include "msptypes.h"

#ifdef PLATFORM_ESP8266
#include "ESP8266_WebUpdate.h"
#include "ESP8266_hwTimer.h"
#endif

#ifdef PLATFORM_STM32
#include "STM32_UARTinHandler.h"
#include "STM32_hwTimer.h"
#endif

bool InBindingMode = false;

void EnterBindingMode();
void ExitBindingMode();

//// CONSTANTS ////
#define BUTTON_SAMPLE_INTERVAL 150
#define WEB_UPDATE_PRESS_INTERVAL 2000 // hold button for 2 sec to enable webupdate mode
#define BUTTON_RESET_INTERVAL 4000     //hold button for 4 sec to reboot RX
#define WEB_UPDATE_LED_FLASH_INTERVAL 25
#define SEND_LINK_STATS_TO_FC_INTERVAL 100
///////////////////

hwTimer hwTimer;
SX127xDriver Radio;
CRSF crsf(Serial); //pass a serial port object to the class for it to use

/// Filters ////////////////
LPF LPF_PacketInterval(3);
LPF LPF_Offset(3);
LPF LPF_FreqError(3);
LPF LPF_UplinkRSSI(5);
////////////////////////////

uint8_t scanIndex = 0;

int32_t HWtimerError;
int32_t Offset;

bool LED = false;

//// Variables Relating to Button behaviour ////
bool buttonPrevValue = true; //default pullup
bool buttonDown = false;     //is the button current pressed down?
uint32_t buttonLastSampled = 0;
uint32_t buttonLastPressed = 0;

bool webUpdateMode = false;
uint32_t webUpdateLedFlashIntervalLast;
///////////////////////////////////////////////

volatile uint8_t NonceRXlocal = 0; // nonce that we THINK we are up to.

bool alreadyFHSS = false;
bool alreadyTLMresp = false;

//////////////////////////////////////////////////////////////

///////Variables for Telemetry and Link Quality///////////////
uint32_t LastValidPacketMicros = 0;
uint32_t LastValidPacketPrevMicros = 0; //Previous to the last valid packet (used to measure the packet interval)
uint32_t LastValidPacket = 0;           //Time the last valid packet was recv

uint32_t SendLinkStatstoFCintervalLastSent = 0;

int16_t RFnoiseFloor; //measurement of the current RF noise floor
///////////////////////////////////////////////////////////////

uint32_t PacketIntervalError;
uint32_t PacketInterval;

/// Variables for Sync Behaviour ////
uint32_t RFmodeLastCycled = 0;
///////////////////////////////////////

void ICACHE_RAM_ATTR getRFlinkInfo()
{
    int8_t LastRSSI = Radio.GetLastPacketRSSI();
    crsf.PackedRCdataOut.ch15 = UINT10_to_CRSF(map(LastRSSI, -100, -50, 0, 1023));
    crsf.PackedRCdataOut.ch14 = UINT10_to_CRSF(fmap(linkQuality, 0, 100, 0, 1023));

    int32_t rssiDBM = LPF_UplinkRSSI.update(Radio.GetLastPacketRSSI());
    // our rssiDBM is currently in the range -128 to 98, but BF wants a value in the range
    // 0 to 255 that maps to -1 * the negative part of the rssiDBM, so cap at 0.
    if (rssiDBM > 0)
        rssiDBM = 0;
    crsf.LinkStatistics.uplink_RSSI_1 = -1 * rssiDBM;   // to match BF

    crsf.LinkStatistics.uplink_RSSI_2 = 0;
    crsf.LinkStatistics.uplink_SNR = Radio.GetLastPacketSNR() * 10;
    crsf.LinkStatistics.uplink_Link_quality = linkQuality;
    crsf.LinkStatistics.rf_Mode = 4 - ExpressLRS_currAirRate->enum_rate;

    //Serial.println(crsf.LinkStatistics.uplink_RSSI_1);
}

void ICACHE_RAM_ATTR HandleFHSS()
{
    if (InBindingMode) {
        return;
    }
    
    uint8_t modresult = (NonceRXlocal + 1) % ExpressLRS_currAirRate->FHSShopInterval;
    if (modresult != 0)
    {
        return;
    }

    linkQuality = getRFlinkQuality();
    if (connectionState != disconnected) // don't hop if we lost
    {
        Radio.SetFrequency(FHSSgetNextFreq());
        Radio.RXnb();
        //crsf.sendLinkStatisticsToFC();
    }
}

void ICACHE_RAM_ATTR HandleSendTelemetryResponse()
{
    if (connectionState != connected)
    {
        return; // don't bother sending tlm if disconnected
    }
    uint8_t modresult = (NonceRXlocal + 1) % TLMratioEnumToValue(ExpressLRS_currAirRate->TLMinterval);
    if (modresult != 0)
    {
        return;
    }

    Radio.TXdataBuffer[0] = (DeviceAddr << 2) + 0b11; // address + tlm packet
    Radio.TXdataBuffer[1] = CRSF_FRAMETYPE_LINK_STATISTICS;

    // OpenTX hard codes "rssi" warnings to the LQ sensor for crossfire, so the
    // rssi we send is for display only.
    // OpenTX treats the rssi values as signed.

    uint8_t openTxRSSI = crsf.LinkStatistics.uplink_RSSI_1;
    // truncate the range to fit into OpenTX's 8 bit signed value
    if (openTxRSSI>127)
        openTxRSSI = 127;
    // convert to 8 bit signed value in the negative range (-128 to 0)
    openTxRSSI = 255-openTxRSSI;
    Radio.TXdataBuffer[2] = openTxRSSI;

    Radio.TXdataBuffer[3] = (crsf.TLMbattSensor.voltage & 0xFF00) >> 8;
    Radio.TXdataBuffer[4] = crsf.LinkStatistics.uplink_SNR;
    Radio.TXdataBuffer[5] = crsf.LinkStatistics.uplink_Link_quality;
    Radio.TXdataBuffer[6] = (crsf.TLMbattSensor.voltage & 0x00FF);

    uint8_t crc = CalcCRC(Radio.TXdataBuffer, 7) + CRCCaesarCipher;
    Radio.TXdataBuffer[7] = crc;
    Radio.TXnb(Radio.TXdataBuffer, 8);
    addPacketToLQ(); // Adds packet to LQ otherwise an artificial drop in LQ is seen due to sending TLM.
}

void ICACHE_RAM_ATTR HWtimerCallback()
{

    if (alreadyFHSS == true)
    {
        alreadyFHSS = false;
    }
    else
    {
        HandleFHSS();
    }

    incrementLQArray();
    HandleSendTelemetryResponse();

    NonceRXlocal++;
}

void ICACHE_RAM_ATTR LostConnection()
{
    if (connectionState == disconnected)
    {
        return; // Already disconnected
    }

    connectionStatePrev = connectionState;
    connectionState = disconnected; //set lost connection
    LPF_FreqError.init(0);

    digitalWrite(GPIO_PIN_LED, 0);        // turn off led
    if (!InBindingMode) {
        Radio.SetFrequency(GetInitialFreq()); // in conn lost state we always want to listen on freq index 0
    }
    
    Serial.println("lost conn");

#ifdef PLATFORM_STM32

    digitalWrite(GPIO_PIN_LED_GREEN, LOW);
#endif
}

void ICACHE_RAM_ATTR TentativeConnection()
{
    connectionStatePrev = connectionState;
    connectionState = tentative;
    Serial.println("tentative conn");
}

void ICACHE_RAM_ATTR GotConnection()
{
    if (connectionState == connected)
    {
        return; // Already connected
    }

    connectionStatePrev = connectionState;
    connectionState = connected; //we got a packet, therefore no lost connection

    RFmodeLastCycled = millis();   // give another 3 sec for loc to occur.
    digitalWrite(GPIO_PIN_LED, 1); // turn on led
    Serial.println("got conn");

#ifdef PLATFORM_STM32

    digitalWrite(GPIO_PIN_LED_GREEN, HIGH);

#endif
}

void ICACHE_RAM_ATTR UnpackChannelData_11bit()
{
    crsf.PackedRCdataOut.ch0 = (Radio.RXdataBuffer[1] << 3) + ((Radio.RXdataBuffer[5] & 0b11100000) >> 5);
    crsf.PackedRCdataOut.ch1 = (Radio.RXdataBuffer[2] << 3) + ((Radio.RXdataBuffer[5] & 0b00011100) >> 2);
    crsf.PackedRCdataOut.ch2 = (Radio.RXdataBuffer[3] << 3) + ((Radio.RXdataBuffer[5] & 0b00000011) << 1) + (Radio.RXdataBuffer[6] & 0b10000000 >> 7);
    crsf.PackedRCdataOut.ch3 = (Radio.RXdataBuffer[4] << 3) + ((Radio.RXdataBuffer[6] & 0b01110000) >> 4);
#ifdef One_Bit_Switches
    crsf.PackedRCdataOut.ch4 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00001000);
    crsf.PackedRCdataOut.ch5 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00000100);
    crsf.PackedRCdataOut.ch6 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00000010);
    crsf.PackedRCdataOut.ch7 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00000001);
#endif
}

void ICACHE_RAM_ATTR UnpackChannelData_10bit()
{
    crsf.PackedRCdataOut.ch0 = UINT10_to_CRSF((Radio.RXdataBuffer[1] << 2) + ((Radio.RXdataBuffer[5] & 0b11000000) >> 6));
    crsf.PackedRCdataOut.ch1 = UINT10_to_CRSF((Radio.RXdataBuffer[2] << 2) + ((Radio.RXdataBuffer[5] & 0b00110000) >> 4));
    crsf.PackedRCdataOut.ch2 = UINT10_to_CRSF((Radio.RXdataBuffer[3] << 2) + ((Radio.RXdataBuffer[5] & 0b00001100) >> 2));
    crsf.PackedRCdataOut.ch3 = UINT10_to_CRSF((Radio.RXdataBuffer[4] << 2) + ((Radio.RXdataBuffer[5] & 0b00000011) >> 0));
}

void ICACHE_RAM_ATTR UnpackMSPData()
{
    mspPacket_t packet;
    packet.reset();
    packet.makeCommand();
    packet.flags = 0;
    packet.function = Radio.RXdataBuffer[1];
    packet.addByte(Radio.RXdataBuffer[3]);
    packet.addByte(Radio.RXdataBuffer[4]);
    packet.addByte(Radio.RXdataBuffer[5]);
    packet.addByte(Radio.RXdataBuffer[6]);

    if (packet.function = 0x01) {
        UID[2] = packet.readByte();
        UID[3] = packet.readByte();
        UID[4] = packet.readByte();
        UID[5] = packet.readByte();
        ExitBindingMode();
    }
    else {
        crsf.sendMSPFrameToFC(&packet);
    }
}

void ICACHE_RAM_ATTR ProcessRFPacket()
{
    uint8_t calculatedCRC = CalcCRC(Radio.RXdataBuffer, 7) + CRCCaesarCipher;
    uint8_t inCRC = Radio.RXdataBuffer[7];
    uint8_t type = Radio.RXdataBuffer[0] & 0b11;
    uint8_t packetAddr = (Radio.RXdataBuffer[0] & 0b11111100) >> 2;

    if (inCRC != calculatedCRC)
    {
        Serial.println("CRC error on RF packet");
        return;
    }

    if (packetAddr != DeviceAddr)
    {
        Serial.println("Wrong device address on RF packet");
        return;
    }

    LastValidPacketPrevMicros = LastValidPacketMicros;
    LastValidPacketMicros = micros();
    LastValidPacket = millis();

    getRFlinkInfo();

    switch (type)
    {
    case RC_DATA_PACKET: //Standard RC Data Packet
        #if defined SEQ_SWITCHES
        UnpackChannelDataSeqSwitches(&Radio, &crsf);
        #elif defined HYBRID_SWITCHES_8
        UnpackChannelDataHybridSwitches8(&Radio, &crsf);
        #else
        UnpackChannelData_11bit();
        #endif
        // crsf.sendRCFrameToFC();
        break;

    case MSP_DATA_PACKET:
        UnpackMSPData();
        break;

    case TLM_PACKET: //telemetry packet from master

        // not implimented yet
        break;

    case SYNC_PACKET: //sync packet from master
        if (Radio.RXdataBuffer[4] == UID[3] && Radio.RXdataBuffer[5] == UID[4] && Radio.RXdataBuffer[6] == UID[5])
        {
            if (connectionState == disconnected)
            {
                TentativeConnection();
            }

            if (connectionState == tentative && NonceRXlocal == Radio.RXdataBuffer[2] && FHSSgetCurrIndex() == Radio.RXdataBuffer[1])
            {
                GotConnection();
            }

            // if (ExpressLRS_currAirRate->enum_rate == !(expresslrs_RFrates_e)(Radio.RXdataBuffer[2] & 0b00001111))
            // {
            //     Serial.println("update air rate");
            //     SetRFLinkRate(ExpressLRS_AirRateConfig[Radio.RXdataBuffer[3]]);
            //     ExpressLRS_currAirRate = ExpressLRS_AirRateConfig[Radio.RXdataBuffer[3]];
            // }

            FHSSsetCurrIndex(Radio.RXdataBuffer[1]);
            NonceRXlocal = Radio.RXdataBuffer[2];
        }
        break;

    default: // code to be executed if n doesn't match any cases
        break;
    }

    addPacketToLQ();

    HWtimerError = ((micros() - hwTimer.LastCallbackMicrosTick) % ExpressLRS_currAirRate->interval);
    Offset = LPF_Offset.update(HWtimerError - (ExpressLRS_currAirRate->interval >> 1)); //crude 'locking function' to lock hardware timer to transmitter, seems to work well enough
    hwTimer.phaseShift(uint32_t((Offset >> 4) + timerOffset));

    if (((NonceRXlocal + 1) % ExpressLRS_currAirRate->FHSShopInterval) == 0) //premept the FHSS if we already know we'll have to do it next timer tick.
    {
        int32_t freqerror = LPF_FreqError.update(Radio.GetFrequencyError());
        //Serial.print(freqerror);
        //Serial.print(" : ");

        if (freqerror > 0)
        {
            if (FreqCorrection < FreqCorrectionMax)
            {
                FreqCorrection += 61; //min freq step is ~ 61hz
            }
            else
            {
                FreqCorrection = FreqCorrectionMax;
                Serial.println("Max pos reasontable freq offset correction limit reached!");
            }
        }
        else
        {
            if (FreqCorrection > FreqCorrectionMin)
            {
                FreqCorrection -= 61; //min freq step is ~ 61hz
            }
            else
            {
                FreqCorrection = FreqCorrectionMin;
                Serial.println("Max neg reasontable freq offset correction limit reached!");
            }
        }

        Radio.setPPMoffsetReg(FreqCorrection);

        //Serial.println(FreqCorrection);

        HandleFHSS();
        alreadyFHSS = true;
    }
}

void beginWebsever()
{
#ifdef PLATFORM_STM32
#else
    Radio.StopContRX();
    hwTimer.stop();

    BeginWebUpdate();
    webUpdateMode = true;
#endif
}

void ICACHE_RAM_ATTR sampleButton()
{
    bool buttonValue = digitalRead(GPIO_PIN_BUTTON);

    if (buttonValue == false && buttonPrevValue == true)
    { //falling edge
        buttonLastPressed = millis();
        buttonDown = true;
        // Serial.println("Manual Start");
        // Radio.SetFrequency(GetInitialFreq());
        // Radio.RXnb();
    }

    if (buttonValue == true && buttonPrevValue == false) //rising edge
    {
        buttonDown = false;
    }

    if ((millis() > buttonLastPressed + WEB_UPDATE_PRESS_INTERVAL) && buttonDown) // button held down
    {
        EnterBindingMode();
    }

    if ((millis() > buttonLastPressed + BUTTON_RESET_INTERVAL) && buttonDown)
    {
#ifdef PLATFORM_ESP8266
        ESP.restart();
#endif
    }

    buttonPrevValue = buttonValue;
}

void ICACHE_RAM_ATTR SetRFLinkRate(expresslrs_RFrates_e rate) // Set speed of RF link (hz)
{
    const expresslrs_mod_settings_s *const mode = get_elrs_airRateConfig(rate);
    Radio.StopContRX();
    Radio.Config(mode->bw, mode->sf, mode->cr, Radio.currFreq, Radio._syncWord);
    ExpressLRS_currAirRate = mode;
    hwTimer.updateInterval(mode->interval);
    LPF_PacketInterval.init(mode->interval);
    //LPF_Offset.init(0);
    //InitHarwareTimer();
    Radio.RXnb();
}

void setup()
{
#ifdef PLATFORM_STM32
    Serial.setTx(GPIO_PIN_RCSIGNAL_TX);
    Serial.setRx(GPIO_PIN_RCSIGNAL_RX);
    Serial.begin(420000);
    crsf.InitSerial();
#endif

#ifdef PLATFORM_ESP8266
    Serial.begin(420000);

#endif
    // Serial.begin(230400); // for linux debugging

    Serial.println("Module Booting...");
    pinMode(GPIO_PIN_LED, OUTPUT);

#ifdef PLATFORM_STM32
    pinMode(GPIO_PIN_LED_GREEN, OUTPUT);
#endif
    pinMode(GPIO_PIN_BUTTON, INPUT_PULLUP);

#ifdef Regulatory_Domain_AU_915
    Serial.println("Setting 915MHz Mode");
    Radio.RFmodule = RFMOD_SX1276; //define radio module here
#elif defined Regulatory_Domain_FCC_915
    Serial.println("Setting 915MHz Mode");
    Radio.RFmodule = RFMOD_SX1276; //define radio module here
#elif defined Regulatory_Domain_EU_868
    Serial.println("Setting 868MHz Mode");
    Radio.RFmodule = RFMOD_SX1276; //define radio module here
#elif defined Regulatory_Domain_AU_433 || defined Regulatory_Domain_EU_433
    Serial.println("Setting 433MHz Mode");
    Radio.RFmodule = RFMOD_SX1278; //define radio module here
#else
#error No regulatory domain defined, please define one in common.h
#endif

    FHSSrandomiseFHSSsequence();
    if (!InBindingMode) {
        Radio.SetFrequency(GetInitialFreq());
    }
    

    //Radio.SetSyncWord(0x122);

    Radio.Begin();

    Radio.SetOutputPower(0b1111); //default is max power (17dBm for RX)

    RFnoiseFloor = MeasureNoiseFloor();
    Serial.print("RF noise floor: ");
    Serial.print(RFnoiseFloor);
    Serial.println("dBm");

    Radio.RXdoneCallback1 = &ProcessRFPacket;

    Radio.TXdoneCallback1 = &Radio.RXnb;

    crsf.Begin();
    hwTimer.callbackTock = &HWtimerCallback;
    hwTimer.init();

    SetRFLinkRate(RATE_200HZ);
    hwTimer.init();
}

void loop()
{
    if (millis() > (RFmodeLastCycled + ExpressLRS_currAirRate->RFmodeCycleInterval + ((connectionState == tentative) ? ExpressLRS_currAirRate->RFmodeCycleAddtionalTime : 0))) // connection = tentative we add alittle delay
    {
        if ((connectionState == disconnected) && !webUpdateMode)
        {
            if (!InBindingMode) {
                Radio.SetFrequency(GetInitialFreq());
            
            
            SetRFLinkRate((expresslrs_RFrates_e)(scanIndex % RATE_25HZ)); //switch between 200hz, 100hz, 50hz, rates
            LQreset();
            digitalWrite(GPIO_PIN_LED, LED);
            LED = !LED;
            Serial.println(ExpressLRS_currAirRate->interval);
            scanIndex++;
            }
        }
        RFmodeLastCycled = millis();
    }

    if (millis() > (LastValidPacket + ExpressLRS_currAirRate->RFmodeCycleAddtionalTime)) // check if we lost conn.
    {
        LostConnection();
    }

    if ((millis() > (SendLinkStatstoFCintervalLastSent + SEND_LINK_STATS_TO_FC_INTERVAL)) && connectionState != disconnected)
    {
        // crsf.sendLinkStatisticsToFC();
        SendLinkStatstoFCintervalLastSent = millis();
    }

    if (millis() > (buttonLastSampled + BUTTON_SAMPLE_INTERVAL))
    {
        sampleButton();
        buttonLastSampled = millis();
    }

#ifdef Auto_WiFi_On_Boot
    if ((connectionState == disconnected) && !webUpdateMode && millis() > 10000 && millis() < 11000)
    {
        beginWebsever();
    }
#endif

#ifdef PLATFORM_STM32
    STM32_RX_HandleUARTin();
#endif

#ifdef PLATFORM_ESP8266
    if (webUpdateMode)
    {
        HandleWebUpdate();
        if (millis() > WEB_UPDATE_LED_FLASH_INTERVAL + webUpdateLedFlashIntervalLast)
        {
            digitalWrite(GPIO_PIN_LED, LED);
            LED = !LED;
            webUpdateLedFlashIntervalLast = millis();
        }
    }
#endif
}

void EnterBindingMode()
{
    // Set UID to special binding values
    UID[0] = BindingUID[0];
    UID[1] = BindingUID[1];
    UID[2] = BindingUID[2];
    UID[3] = BindingUID[3];
    UID[4] = BindingUID[4];
    UID[5] = BindingUID[5];

    CRCCaesarCipher = UID[4];
    DeviceAddr = UID[5] & 0b111111;

    // Start attempting to bind
    // Lock the RF rate and freq while binding
    InBindingMode = true;
    SetRFLinkRate(RATE_50HZ);
    Radio.SetFrequency(FHSSfreqs[0]);

    Serial.print("Entered binding mode at freq = ");
    Serial.print(FHSSfreqs[0]);
    Serial.print(" and rfmode = ");
    Serial.print(ExpressLRS_currAirRate->rate);
    Serial.println("Hz");
}

void ExitBindingMode()
{
  CRCCaesarCipher = UID[4];
  DeviceAddr = UID[5] & 0b111111;

  // Revert to original packet rate
  // and go to initial freq
  InBindingMode = false;
  SetRFLinkRate(RATE_200HZ);
  Radio.SetFrequency(GetInitialFreq());

  Serial.print("Exit binding mode at freq = ");
  Serial.print(FHSSfreqs[0]);
  Serial.print(" and rfmode = ");
  Serial.print(ExpressLRS_currAirRate->rate);
  Serial.println("Hz");

  Serial.print("New UID = ");
  Serial.print(UID[0]);
  Serial.print(", ");
  Serial.print(UID[1]);
  Serial.print(", ");
  Serial.print(UID[2]);
  Serial.print(", ");
  Serial.print(UID[3]);
  Serial.print(", ");
  Serial.print(UID[4]);
  Serial.print(", ");
  Serial.println(UID[5]);
}