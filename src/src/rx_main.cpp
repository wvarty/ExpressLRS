#include <Arduino.h>
#include "targets.h"
#include "utils.h"
#include "common.h"
#include "LowPassFilter.h"
#include "LoRaRadioLib.h"
#include "CRSF.h"
#include "FHSS.h"
#include "debug.h"
#include "rx_LinkQuality.h"

#ifdef PLATFORM_ESP8266
#include <EEPROM.h>
#include "ESP8266_WebUpdate.h"
#endif
#ifdef PLATFORM_STM32
#include "../lib/Arduino_Core_STM32-master/libraries/EEPROM/src/EEPROM.h"
#include "STM32_UARTinHandler.h"
#endif

#include "button.h"
#include "errata.h"

// expresslrs packet header types
// 00 -> standard 4 channel data packet
// 01 -> switch data packet
// 11 -> tlm packet
// 10 -> sync packet with hop data
#define RC_DATA_PACKET      0b00
#define SWITCH_DATA_PACKET  0b01
#define TLM_PACKET          0b11
#define SYNC_PACKET         0b10

// LED blink rates for different modes
#define LED_BINDING_INTERVAL 100
#define LED_DISCONNECTED_INTERVAL 1000
#define LED_WEB_UPDATE_INTERVAL 25

SX127xDriver Radio;
CRSF crsf(Serial); //pass a serial port object to the class for it to use
Button button;

//Filters//
LPF LPF_PacketInterval(3);
LPF LPF_Offset(2);

///forward defs///
void SetRFLinkRate(expresslrs_mod_settings_s mode);
void InitOStimer();
void OStimerSetCallback(void (*CallbackFunc)(void));
void OStimerReset();
void OStimerUpdateInterval(uint32_t Interval);

void InitHarwareTimer();
void StopHWtimer();
void HWtimerSetCallback(void (*CallbackFunc)(void));
void HWtimerSetCallback90(void (*CallbackFunc)(void));
void HWtimerUpdateInterval(uint32_t Interval);
uint32_t ICACHE_RAM_ATTR HWtimerGetlastCallbackMicros();
uint32_t ICACHE_RAM_ATTR HWtimerGetlastCallbackMicros90();
void ICACHE_RAM_ATTR HWtimerPhaseShift(int16_t Offset);
uint32_t ICACHE_RAM_ATTR HWtimerGetIntervalMicros();

bool InBindingMode = false;
void EnterBindingMode();
void ExitBindingMode();
void CancelBindingMode();

void ReadMacFromFlash();
void WriteMacToFlash();

void UpdateLEDState(bool forceOff = false);

void UpdateConnectionState(connectionState_e state);

uint8_t scanIndex = 0;

uint8_t prevAirRate = 0;
uint8_t currAirRate = 0;

uint32_t MeasuredHWtimerInterval;
int32_t HWtimerError;
int32_t HWtimerError90;
int16_t Offset;
int16_t Offset90;
uint32_t SerialDebugPrintInterval = 250;
uint32_t LastSerialDebugPrint = 0;

uint32_t RFmodeLastCycled = 0;
uint32_t RFmodeCycleInterval = 1000;

uint32_t LEDLastCycled = 0;
uint32_t LEDCycleInterval = 1000;

bool LED = false;
bool webUpdateMode = false;

volatile uint8_t NonceRXlocal = 0; // nonce that we THINK we are up to.

bool alreadyFHSS = false;
bool alreadyTLMresp = false;

//////////////////////////////////////////////////////////////

///////Variables for Telemetry and Link Quality///////////////
int packetCounter = 0;
int CRCerrorCounter = 0;
float CRCerrorRate = 0;
uint32_t PacketRateLastChecked = 0;
uint32_t PacketRateInterval = 1000;

float PacketRate = 0.0;

uint32_t LastValidPacketMicros = 0;
uint32_t LastValidPacketPrevMicros = 0; //Previous to the last valid packet (used to measure the packet interval)
uint32_t LastValidPacket = 0;           //Time the last valid packet was recv

uint32_t SendLinkStatstoFCinterval = 100;
uint32_t SendLinkStatstoFCintervalLastSent = 0;
///////////////////////////////////////////////////////////////

uint32_t PacketIntervalError;
uint32_t PacketInterval;

/// Variables for Sync Behaviour ////
uint32_t RFmodeLastCycled = 0;
const uint32_t RFmodeCycleAddtionalTime[3] = {1000, 2000, 4000}; //After we have a tentative sync we wait this long in addtion before jumping to different RF mode again.
const uint32_t RFmodeCycleInterval[3] = {1000, 1500, 2000};
///////////////////////////////////////

void ICACHE_RAM_ATTR GenerateSyncPacketData()
{
    uint8_t PacketHeaderAddr;
    PacketHeaderAddr = (DeviceAddr << 2) + 0b10;
    Radio.TXdataBuffer[0] = PacketHeaderAddr;
    Radio.TXdataBuffer[1] = FHSSgetCurrIndex();
    Radio.TXdataBuffer[2] = NonceRXlocal;
    Radio.TXdataBuffer[3] = ExpressLRS_currAirRate.enum_rate;
}

void ICACHE_RAM_ATTR getRFlinkInfo()
{
    int8_t LastRSSI = Radio.GetLastPacketRSSI();
    crsf.PackedRCdataOut.ch15 = UINT10_to_CRSF(map(LastRSSI, -100, -50, 0, 1023));
    crsf.PackedRCdataOut.ch14 = UINT10_to_CRSF(fmap(linkQuality, 0, 100, 0, 1023));

    crsf.LinkStatistics.uplink_RSSI_1 = 120 + Radio.GetLastPacketRSSI();
    crsf.LinkStatistics.uplink_RSSI_2 = 0;
    crsf.LinkStatistics.uplink_SNR = Radio.GetLastPacketSNR() * 10;
    crsf.LinkStatistics.uplink_Link_quality = linkQuality;

    #ifndef DEBUG
    crsf.sendLinkStatisticsToFC();
    #endif
}

void ICACHE_RAM_ATTR HandleFHSS()
{
    if (FreqLocked) {
        return;
    }
    
    uint8_t modresult = (NonceRXlocal + 1) % ExpressLRS_currAirRate.FHSShopInterval;

    if (modresult == 0)
    {
        linkQuality = getRFlinkQuality();
        if (connectionState != disconnected) // don't hop if we lost
        {
            Radio.SetFrequency(FHSSgetNextFreq());
            Radio.RXnb();
            #ifndef DEBUG
            crsf.sendLinkStatisticsToFC();
            #endif
        }
    }
}

void ICACHE_RAM_ATTR HandleSendTelemetryResponse()
{
    if (connectionState != connected || InBindingMode)
    {
        // Don't bother sending tlm if disconnected or binding
        return;
    }

    uint8_t modresult = (NonceRXlocal + 1) % ExpressLRS_currAirRate.TLMinterval;

    if (modresult == 0)
    {
        Radio.TXdataBuffer[0] = (DeviceAddr << 2) + 0b11; // address + tlm packet
        Radio.TXdataBuffer[1] = CRSF_FRAMETYPE_LINK_STATISTICS;
        Radio.TXdataBuffer[2] = crsf.LinkStatistics.uplink_RSSI_1;
        Radio.TXdataBuffer[3] = 0;
        Radio.TXdataBuffer[4] = crsf.LinkStatistics.uplink_SNR;
        Radio.TXdataBuffer[5] = crsf.LinkStatistics.uplink_Link_quality;

        uint8_t crc = CalcCRC(Radio.TXdataBuffer, 7) + CRCCaesarCipher;
        Radio.TXdataBuffer[7] = crc;
        Radio.TXnb(Radio.TXdataBuffer, 8);
        addPacketToLQ(); // Adds packet to LQ otherwise an artificial drop in LQ is seen due to sending TLM.
    }
}

void ICACHE_RAM_ATTR Test90()
{
    incrementLQArray();

    if (alreadyFHSS == true)
    {

        alreadyFHSS = false;
    }
    else
    {
        HandleFHSS();
    }

    HandleSendTelemetryResponse();

    NonceRXlocal++;
}

void ICACHE_RAM_ATTR Test()
{
    MeasuredHWtimerInterval = micros() - HWtimerGetlastCallbackMicros();
}

void ICACHE_RAM_ATTR LostConnection()
{
    if (connectionState != disconnected)
    {
        connectionStatePrev = connectionState;
        connectionState = disconnected; //set lost connection

        UpdateLEDState(true); // turn off led
        Radio.SetFrequency(GetInitialFreq()); // in conn lost state we always want to listen on freq index 0
        DEBUG_PRINTLN("lost conn");
    }
}

void ICACHE_RAM_ATTR TentativeConnection()
{
    UpdateConnectionState(tentative);
    DEBUG_PRINTLN("tentative conn");
}

void ICACHE_RAM_ATTR GotConnection()
{
    if (connectionState != connected)
    {
        UpdateConnectionState(connected); //we got a packet, therefore no lost connection

        RFmodeLastCycled = millis();   // give another 3 sec for loc to occur.
        UpdateLEDState(); // turn on led
        DEBUG_PRINTLN("got conn");
    }
}

void ICACHE_RAM_ATTR UnpackChannelData_11bit()
{
    crsf.PackedRCdataOut.ch0 = (Radio.RXdataBuffer[1] << 3) + (Radio.RXdataBuffer[5] & 0b11100000 >> 5);
    crsf.PackedRCdataOut.ch1 = (Radio.RXdataBuffer[2] << 3) + (Radio.RXdataBuffer[5] & 0b00011100 >> 2);
    crsf.PackedRCdataOut.ch2 = (Radio.RXdataBuffer[3] << 3) + (Radio.RXdataBuffer[5] & 0b00000011 << 1) + (Radio.RXdataBuffer[6] & 0b10000000 >> 7);
    crsf.PackedRCdataOut.ch3 = (Radio.RXdataBuffer[4] << 3) + (Radio.RXdataBuffer[6] & 0b01110000 >> 4);
#ifdef One_Bit_Switches
    crsf.PackedRCdataOut.ch4 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00001000);
    crsf.PackedRCdataOut.ch5 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00000100);
    crsf.PackedRCdataOut.ch6 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00000010);
    crsf.PackedRCdataOut.ch7 = BIT_to_CRSF(Radio.RXdataBuffer[6] & 0b00000001);
#endif
}

void ICACHE_RAM_ATTR UnpackChannelData_10bit()
{
    crsf.PackedRCdataOut.ch0 = UINT10_to_CRSF((Radio.RXdataBuffer[1] << 2) + (Radio.RXdataBuffer[5] & 0b11000000 >> 6));
    crsf.PackedRCdataOut.ch1 = UINT10_to_CRSF((Radio.RXdataBuffer[2] << 2) + (Radio.RXdataBuffer[5] & 0b00110000 >> 4));
    crsf.PackedRCdataOut.ch2 = UINT10_to_CRSF((Radio.RXdataBuffer[3] << 2) + (Radio.RXdataBuffer[5] & 0b00001100 >> 2));
    crsf.PackedRCdataOut.ch3 = UINT10_to_CRSF((Radio.RXdataBuffer[4] << 2) + (Radio.RXdataBuffer[5] & 0b00000011 >> 0));
}

void ICACHE_RAM_ATTR UnpackSwitchData()
{
    crsf.PackedRCdataOut.ch4 = SWITCH3b_to_CRSF((uint16_t)(Radio.RXdataBuffer[1] & 0b11100000) >> 5); //unpack the byte structure, each switch is stored as a possible 8 states (3 bits). we shift by 2 to translate it into the 0....1024 range like the other channel data.
    crsf.PackedRCdataOut.ch5 = SWITCH3b_to_CRSF((uint16_t)(Radio.RXdataBuffer[1] & 0b00011100) >> 2);
    crsf.PackedRCdataOut.ch6 = SWITCH3b_to_CRSF((uint16_t)((Radio.RXdataBuffer[1] & 0b00000011) << 1) + ((Radio.RXdataBuffer[2] & 0b10000000) >> 7));
    crsf.PackedRCdataOut.ch7 = SWITCH3b_to_CRSF((uint16_t)((Radio.RXdataBuffer[2] & 0b01110000) >> 4));
}

void ICACHE_RAM_ATTR ProcessRFPacket()
{
    uint8_t calculatedCRC = CalcCRC(Radio.RXdataBuffer, 7) + CRCCaesarCipher;
    uint8_t inCRC = Radio.RXdataBuffer[7];
    uint8_t type = Radio.RXdataBuffer[0] & 0b11;
    uint8_t packetAddr = (Radio.RXdataBuffer[0] & 0b11111100) >> 2;

    // DEBUG_PRINT(NonceRXlocal);
    // DEBUG_PRINT(" ");
    // DEBUG_PRINT(inCRC);
    // DEBUG_PRINT(" = ");
    // DEBUG_PRINTLN(calculatedCRC);

    if (inCRC != calculatedCRC) {
        DEBUG_PRINTLN("crc failed");
        DEBUG_PRINT(calculatedCRC);
        DEBUG_PRINT(" != ");
        DEBUG_PRINTLN(inCRC);
        CRCerrorCounter++;

        DEBUG_PRINTLN(Radio.RXdataBuffer[4]);
        DEBUG_PRINTLN(Radio.RXdataBuffer[5]);
        DEBUG_PRINTLN(Radio.RXdataBuffer[6]);
        return;
    }

    if (packetAddr != DeviceAddr) {
        DEBUG_PRINTLN("wrong address");
        return;
    }

    LastValidPacketPrevMicros = LastValidPacketMicros;
    LastValidPacketMicros = micros();
    HWtimerError = micros() - HWtimerGetlastCallbackMicros();
    packetCounter++;

    getRFlinkInfo();

    switch (type)
    {
    case RC_DATA_PACKET: //Standard RC Data Packet
        UnpackChannelData_11bit();
        #ifndef DEBUG
        crsf.sendRCFrameToFC();
        #endif
        break;

    case SWITCH_DATA_PACKET: // Switch Data Packet
        //DEBUG_PRINTLN("Switch Packet");
        if ((Radio.RXdataBuffer[3] == Radio.RXdataBuffer[1]) && (Radio.RXdataBuffer[4] == Radio.RXdataBuffer[2])) // extra layer of protection incase the crc and addr headers fail us.
        {
            //GotConnection();
            UnpackSwitchData();
            NonceRXlocal = Radio.RXdataBuffer[5];
            FHSSsetCurrIndex(Radio.RXdataBuffer[6]);
            #ifndef DEBUG
            crsf.sendRCFrameToFC();
            #endif
            //DEBUG_PRINTLN("Switch Pkt");
        }
        break;

    case TLM_PACKET: //telemetry packet from master

        // not implimented yet
        break;

    case SYNC_PACKET:
        if (InBindingMode) {
            ExitBindingMode();
            return;
        }
        if (Radio.RXdataBuffer[4] == TxBaseMac[3] && Radio.RXdataBuffer[5] == TxBaseMac[4] && Radio.RXdataBuffer[6] == TxBaseMac[5])
        {
            if (connectionState == disconnected)
            {
                TentativeConnection();
            }

            if (connectionState == tentative && NonceRXlocal == Radio.RXdataBuffer[2] && FHSSgetCurrIndex() == Radio.RXdataBuffer[1])
            {
                GotConnection();
            }

            // if (ExpressLRS_currAirRate.enum_rate == !(expresslrs_RFrates_e)(Radio.RXdataBuffer[2] & 0b00001111))
            // {
            //     DEBUG_PRINTLN("update air rate");
            //     SetRFLinkRate(ExpressLRS_AirRateConfig[Radio.RXdataBuffer[3]]);
            //     ExpressLRS_currAirRate = ExpressLRS_AirRateConfig[Radio.RXdataBuffer[3]];
            // }

            //NonceRXlocal = (Radio.RXdataBuffer[2] & 0b11110000) >> 4;
            FHSSsetCurrIndex(Radio.RXdataBuffer[1]);
            NonceRXlocal = Radio.RXdataBuffer[2];
        }
        break;

    default: // code to be executed if n doesn't match any cases
        break;
    }

    LastValidPacket = millis();
    addPacketToLQ();

    Offset = LPF_Offset.update(HWtimerError - (ExpressLRS_currAirRate.interval >> 1)) + 0; //crude 'locking function' to lock hardware timer to transmitter, seems to work well enough
    HWtimerPhaseShift((Offset >> 1) + timerOffset);

    if (((NonceRXlocal + 1) % ExpressLRS_currAirRate.FHSShopInterval) == 0) //premept the FHSS if we already know we'll have to do it next timer tick.
    {
        HandleFHSS();
        alreadyFHSS = true;
    }
    // DEBUG_PRINT("Offset: ");
    // DEBUG_PRINT(Offset);
    // DEBUG_PRINT(" LQ: ");
    // DEBUG_PRINTLN(linkQuality);
    // DEBUG_PRINT(":");
    // DEBUG_PRINTLN(PacketInterval);
}

void beginWebsever()
{
#ifndef PLATFORM_STM32
    Radio.StopContRX();
    StopHWtimer();

    BeginWebUpdate();
    webUpdateMode = true;
#endif
}

void ICACHE_RAM_ATTR SetRFLinkRate(expresslrs_mod_settings_s mode) // Set speed of RF link (hz)
{
    Radio.StopContRX();
    Radio.Config(mode.bw, mode.sf, mode.cr, Radio.currFreq, Radio._syncWord);
    ExpressLRS_currAirRate = mode;
    HWtimerUpdateInterval(mode.interval);
    LPF_PacketInterval.init(mode.interval);
    LPF_Offset.init(0);
    InitHarwareTimer();
    Radio.RXnb();
}

void setup()
{
    // Read bound MAC addr from flash
#ifdef PLATFORM_ESP8266
    EEPROM.begin(3);
#endif
    ReadMacFromFlash();
    CRCCaesarCipher = TxBaseMac[4];
    DeviceAddr = TxBaseMac[5] & 0b111111;

#ifdef PLATFORM_STM32
    Serial.setTx(GPIO_PIN_RCSIGNAL_TX);
    Serial.setRx(GPIO_PIN_RCSIGNAL_RX);
    crsf.InitSerial();
#endif

#ifdef DEBUG
    Serial.begin(115200);
    DEBUG_PRINTLN("Booting in DEBUG mode");
#else
    Serial.begin(420000);
#endif

    pinMode(GPIO_PIN_LED, OUTPUT);
    pinMode(GPIO_PIN_BUTTON, INPUT_PULLUP);

#ifdef PLATFORM_STM32
    pinMode(GPIO_PIN_LED_GEEN, OUTPUT);
#endif

#ifdef Regulatory_Domain_AU_915
    DEBUG_PRINTLN("Setting 915MHz Mode");
    Radio.RFmodule = RFMOD_SX1276; //define radio module here
#elif defined Regulatory_Domain_AU_433
    DEBUG_PRINTLN("Setting 433MHz Mode");
    Radio.RFmodule = RFMOD_SX1278; //define radio module here
#endif

    FHSSrandomiseFHSSsequence();
    Radio.SetFrequency(GetInitialFreq());

    Radio.Begin();

    Radio.SetOutputPower(0b1111);

    Radio.RXdoneCallback1 = &ProcessRFPacket;

    Radio.TXdoneCallback1 = &Radio.RXnb;

    crsf.Begin();

    HWtimerSetCallback(&Test);
    HWtimerSetCallback90(&Test90);
    InitHarwareTimer();
    SetRFLinkRate(RF_RATE_200HZ);

    DEBUG_PRINTLN("Setup completed");
    DEBUG_PRINT("Start freq = ");
    DEBUG_PRINTLN(GetInitialFreq());
}

void loop()
{

    if (millis() > (RFmodeLastCycled + RFmodeCycleInterval[scanIndex % 3] + ((connectionState == tentative) ? RFmodeCycleAddtionalTime[scanIndex % 3] : 0))) // connection = tentative we add alittle delay
    {
        if ((connectionState == disconnected) && !webUpdateMode)
        {
            Radio.SetFrequency(GetInitialFreq());
            SetRFLinkRate(ExpressLRS_AirRateConfig[scanIndex % 3]); //switch between 200hz, 100hz, 50hz, rates
            LQreset();
            DEBUG_PRINTLN(ExpressLRS_currAirRate.interval);
            scanIndex++;
        }
        RFmodeLastCycled = millis();
    }

    if (millis() > (LastValidPacket + RFmodeCycleAddtionalTime[(uint8_t)ExpressLRS_currAirRate.enum_rate])) // check if we lost conn.
    {
        LostConnection();
    }

    if ((millis() > (SendLinkStatstoFCintervalLastSent + SendLinkStatstoFCinterval)) && connectionState != disconnected)
    {
        //crsf.sendLinkStatisticsToFC();
        SendLinkStatstoFCintervalLastSent = millis();
    }
    // if (millis() > LastSerialDebugPrint + SerialDebugPrintInterval)
    // { // add stuff here for debug print
    //     LastSerialDebugPrint = millis();
    //     DEBUG_PRINTLN(linkQuality);
    //     if (LostConnection)
    //     {
    //         DEBUG_PRINTLN("-");
    //     }
    //     else
    //     {
    //         DEBUG_PRINTLN("+");
    //     }

    //     DEBUG_PRINT(MeasuredHWtimerInterval);
    //     DEBUG_PRINT(" ");
    //     DEBUG_PRINT(Offset);
    //     DEBUG_PRINT(" ");
    //     DEBUG_PRINT(HWtimerError);

    //     DEBUG_PRINT("----");

    //     DEBUG_PRINT(Offset90);
    //     DEBUG_PRINT(" ");
    //     DEBUG_PRINT(HWtimerError90);
    //     DEBUG_PRINT("----");
    //     DEBUG_PRINTLN(packetCounter);
    // }

    // if (millis() > (PacketRateLastChecked + PacketRateInterval)) //just some debug data
    // {
    //     //     float targetFrameRate;

    //     //     if (ExpressLRS_currAirRate.TLMinterval != 0)
    //     //     {
    //     //         targetFrameRate = ExpressLRS_currAirRate.rate - ((ExpressLRS_currAirRate.rate) * (1.0 / ExpressLRS_currAirRate.TLMinterval));
    //     //     }
    //     //     else
    //     //     {
    //     //         targetFrameRate = ExpressLRS_currAirRate.rate;
    //     //     }

    //     PacketRateLastChecked = millis();
    //     //     PacketRate = (float)packetCounter / (float)(PacketRateInterval);
    //     //     linkQuality = int(((float)PacketRate / (float)targetFrameRate) * 100000.0);
    //     //     if(linkQuality > 99) linkQuality = 99;

    //     //     CRCerrorRate = (((float)CRCerrorCounter / (float)(PacketRateInterval)) * 100);

    //     //     CRCerrorCounter = 0;
    //     //     packetCounter = 0;

    //     //     //DEBUG_PRINTLN(CRCerrorRate);
    //     // }
    //     DEBUG_PRINT(MeasuredHWtimerInterval);
    //     DEBUG_PRINT(" ");
    //     DEBUG_PRINT(Offset);
    //     DEBUG_PRINT(" ");
    //     DEBUG_PRINT(HWtimerError);

    //     DEBUG_PRINT("----");

    //     DEBUG_PRINT(Offset90);
    //     DEBUG_PRINT(" ");
    //     DEBUG_PRINT(HWtimerError90);
    //     DEBUG_PRINT("----");

    //DEBUG_PRINTLN(linkQuality);
    //     //DEBUG_PRINTLN(packetCounter);
    // }

    // DEBUG_PRINT(MeasuredHWtimerInterval);
    // DEBUG_PRINT(" ");
    // DEBUG_PRINT(" ");
    // DEBUG_PRINT(HWtimerError);

    // DEBUG_PRINT("----");

    // DEBUG_PRINT(Offset90);
    // DEBUG_PRINT(" ");
    // DEBUG_PRINT(HWtimerError90);
    // DEBUG_PRINT("----");
    // DEBUG_PRINTLN(packetCounter);
    // delay(200);
    // DEBUG_PRINT("LQ: ");
    // DEBUG_PRINT(linkQuality);
    // DEBUG_PRINT(" Connstate:");
    // DEBUG_PRINTLN(connectionState);

    //yield();

#ifdef Auto_WiFi_On_Boot
    if ((connectionState == disconnected) && !webUpdateMode && !InBindingMode && millis() > 10000 && millis() < 11000)
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
    }
#endif

    button.Sample();
    if (button.IsPressed()) {
        button.Reset();
        if (!InBindingMode) {
            EnterBindingMode();
        }
        else {
            CancelBindingMode();
        }
    }

    UpdateLEDState();
}

void PrintMac()
{
    DEBUG_PRINT("MAC = ");
    DEBUG_PRINT(TxBaseMac[3]);
    DEBUG_PRINT(TxBaseMac[4]);
    DEBUG_PRINTLN(TxBaseMac[5]);
    DEBUG_PRINT("DEV ADDR = ");
    DEBUG_PRINTLN(DeviceAddr);
    DEBUG_PRINT("CRCCaesarCipher = ");
    DEBUG_PRINTLN(CRCCaesarCipher);
}

void EnterBindingMode()
{
    if ((PREVENT_BIND_WHEN_CONNECTED && connectionState == connected) || InBindingMode || webUpdateMode) {
        // Don't enter binding if:
        // - we're already connected
        // - we're already binding
        // - we're in web update mode
        DEBUG_PRINTLN("Cannot enter binding mode!");
        return;
    }

    CRCCaesarCipher = BindingCipher;
    DeviceAddr = BindingAddr;

    // Lock the RF rate and freq to the base freq while binding
    SetRFLinkRate(RF_RATE_200HZ);
    Radio.SetFrequency(919100000);
    FreqLocked = true;

    InBindingMode = true;
    LostConnection = true;

    DEBUG_PRINTLN("=== Entered binding mode ===");
    PrintMac();
}

void ExitBindingMode()
{
    if (!InBindingMode) {
        // Not in binding mode
        return;
    }

    // Read MAC data from TX
    TxBaseMac[3] = Radio.RXdataBuffer[4];
    TxBaseMac[4] = Radio.RXdataBuffer[5];
    TxBaseMac[5] = Radio.RXdataBuffer[6];
    CRCCaesarCipher = TxBaseMac[4];
    DeviceAddr = TxBaseMac[5] & 0b111111;

    // Store MAC in flash for reading on boot
    WriteMacToFlash();

    // Randomise the FHSS seq using the new MAC addr
    // and start at the initial freq
    FHSSrandomiseFHSSsequence();
    FreqLocked = false;
    Radio.SetFrequency(GetInitialFreq());

    InBindingMode = false;
    UpdateConnectionState(disconnected);

    DEBUG_PRINTLN("=== Binding successful ===");
    PrintMac();
}

void CancelBindingMode()
{
    if (!InBindingMode) {
        // Not in binding mode
        return;
    }

    // Binding cancelled
    ReadMacFromFlash();

    // Revert to original cipher and addr
    CRCCaesarCipher = TxBaseMac[4];
    DeviceAddr = TxBaseMac[5] & 0b111111;

    InBindingMode = false;
    UpdateConnectionState(disconnected);

    DEBUG_PRINTLN("=== Binding mode cancelled ===");
    PrintMac();
}

void ReadMacFromFlash()
{
    uint8_t readAddress = 0;

#ifdef PLATFORM_ESP8266
    EEPROM.get(readAddress, TxBaseMac[3]);
    readAddress += sizeof(TxBaseMac[3]);
    EEPROM.get(readAddress, TxBaseMac[4]);
    readAddress += sizeof(TxBaseMac[4]);
    EEPROM.get(readAddress, TxBaseMac[5]);
#endif

#ifdef PLATFORM_STM32
    TxBaseMac[3] = EEPROM.read(readAddress);
    readAddress += sizeof(uint8_t);
    TxBaseMac[4] = EEPROM.read(readAddress);
    readAddress += sizeof(uint8_t);
    TxBaseMac[5] = EEPROM.read(readAddress);
#endif
}

void WriteMacToFlash()
{
    uint8_t writeAddress = 0;

#ifdef PLATFORM_ESP8266
    EEPROM.put(writeAddress, TxBaseMac[3]);
    writeAddress += sizeof(TxBaseMac[3]);
    EEPROM.put(writeAddress, TxBaseMac[4]);
    writeAddress += sizeof(TxBaseMac[4]);
    EEPROM.put(writeAddress, TxBaseMac[5]);
    EEPROM.commit();
#endif

#ifdef PLATFORM_STM32
    EEPROM.write(writeAddress, TxBaseMac[3]);
    writeAddress += sizeof(uint8_t);
    EEPROM.write(writeAddress, TxBaseMac[4]);
    writeAddress += sizeof(uint8_t);
    EEPROM.write(writeAddress, TxBaseMac[5]);
#endif
}


void UpdateLEDState(bool forceOff)
{
    if (forceOff) {
        // Turn the LED off immediately
        // Used on disconnect
        LED = false;
        digitalWrite(GPIO_PIN_LED, LED);
        return;
    }

    if (!InBindingMode && connectionState == connected && !webUpdateMode) {
        // Turn the LED on solid if we're connected
        LED = true;
        digitalWrite(GPIO_PIN_LED, LED);
        return;
    }
    
    // Various flash intervals for other states
    if (InBindingMode) {
        LEDCycleInterval = LED_BINDING_INTERVAL;
    }
    else if (LostConnection && !webUpdateMode) {
        LEDCycleInterval = LED_DISCONNECTED_INTERVAL;
    }
    else if (webUpdateMode) {
        LEDCycleInterval = LED_WEB_UPDATE_INTERVAL;
    }

    if (millis() > (LEDLastCycled + LEDCycleInterval)) {
        LED = !LED;
        digitalWrite(GPIO_PIN_LED, LED);
        LEDLastCycled = millis();
    }
}

void UpdateConnectionState(connectionState_e state)
{
    connectionStatePrev = connectionState;
    connectionState = state;
}
