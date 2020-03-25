#include <Arduino.h>
#include "FIFO.h"
#include "utils.h"
#include "common.h"
#include "LoRaRadioLib.h"
#include "CRSF.h"
#include "FHSS.h"
#include "LED.h"
#include "debug.h"
#include "targets.h"

#ifdef TARGET_EXPRESSLRS_PCB_TX_V3
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

#ifdef TARGET_R9M_TX
#include "DAC.h"
#include "STM32_hwTimer.h"
#include "button.h"
button button;
R9DAC R9DAC;
hwTimer hwTimer;
#endif

//// CONSTANTS ////
#define RX_CONNECTION_LOST_TIMEOUT 1500 // After 1500ms of no TLM response consider that slave has lost connection
#define PACKET_RATE_INTERVAL 500
#define RF_MODE_CYCLE_INTERVAL 1000
#define SWITCH_PACKET_SEND_INTERVAL 200
#define SYNC_PACKET_SEND_INTERVAL_RX_LOST 250  // how often to send the switch data packet (ms) when there is no response from RX
#define SYNC_PACKET_SEND_INTERVAL_RX_CONN 1500 // how often to send the switch data packet (ms) when there we have a connection
///////////////////

String DebugOutput;

/// define some libs to use ///
SX127xDriver Radio;
CRSF crsf;

void TimerExpired();

//// Switch Data Handling ///////
uint32_t SwitchPacketLastSent = 0; //time in ms when the last switch data packet was sent

////////////SYNC PACKET/////////
uint32_t SyncPacketLastSent = 0;

uint32_t LastTLMpacketRecvMillis = 0;
int packetCounteRX_TX = 0;
uint32_t PacketRateLastChecked = 0;
float PacketRate = 0.0;
uint8_t linkQuality = 0;

/// Variables for Sync Behaviour ////
uint32_t RFmodeLastCycled = 0;
///////////////////////////////////////

bool UpdateParamReq = false;

bool Channels5to8Changed = false;

bool ChangeAirRateRequested = false;
bool ChangeAirRateSentUpdate = false;

bool WaitRXresponse = false;

///// Not used in this version /////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t TelemetryWaitBuffer[7] = {0};

uint32_t LinkSpeedIncreaseDelayFactor = 500; // wait for the connection to be 'good' for this long before increasing the speed.
uint32_t LinkSpeedDecreaseDelayFactor = 200; // this long wait this long for connection to be below threshold before dropping speed

uint32_t LinkSpeedDecreaseFirstMetCondition = 0;
uint32_t LinkSpeedIncreaseFirstMetCondition = 0;

uint8_t LinkSpeedReduceSNR = 20;   //if the SNR (times 10) is lower than this we drop the link speed one level
uint8_t LinkSpeedIncreaseSNR = 60; //if the SNR (times 10) is higher than this we increase the link speed
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ICACHE_RAM_ATTR IncreasePower();
void ICACHE_RAM_ATTR DecreasePower();

uint8_t baseMac[6];

void updateRFConnectionState(connectionState_e newState);

void ICACHE_RAM_ATTR ProcessTLMpacket()
{
  uint8_t calculatedCRC = CalcCRC(Radio.RXdataBuffer, 7) + CRCCaesarCipher;
  uint8_t inCRC = Radio.RXdataBuffer[7];
  uint8_t type = Radio.RXdataBuffer[0] & TLM_PACKET;
  uint8_t packetAddr = (Radio.RXdataBuffer[0] & 0b11111100) >> 2;
  uint8_t TLMheader = Radio.RXdataBuffer[1];

  if (packetAddr != DeviceAddr)
  {
    DEBUG_PRINT("[ERROR] TLM device address error: expected ");
    DEBUG_PRINT(DeviceAddr);
    DEBUG_PRINT(" but received ");
    DEBUG_PRINTLN(packetAddr);
    return;
  }

  if ((inCRC != calculatedCRC))
  {
    DEBUG_PRINT("[ERROR] TLM CRC error: expected ");
    DEBUG_PRINT(calculatedCRC);
    DEBUG_PRINT(" but received ");
    DEBUG_PRINTLN(inCRC);
    return;
  }

  packetCounteRX_TX++;

  if (type != TLM_PACKET)
  {
    DEBUG_PRINT("TLM type error: expected ");
    DEBUG_PRINT(TLM_PACKET);
    DEBUG_PRINT(" but received ");
    DEBUG_PRINTLN(type);
    return;
  }

  updateRFConnectionState(connected);
  LastTLMpacketRecvMillis = millis();

  if (TLMheader == CRSF_FRAMETYPE_LINK_STATISTICS)
  {
    crsf.LinkStatistics.uplink_RSSI_1 = Radio.RXdataBuffer[2];
    crsf.LinkStatistics.uplink_RSSI_2 = 0;
    crsf.LinkStatistics.uplink_SNR = Radio.RXdataBuffer[4];
    crsf.LinkStatistics.uplink_Link_quality = Radio.RXdataBuffer[5];

    crsf.LinkStatistics.downlink_SNR = int(Radio.LastPacketSNR * 10);
    crsf.LinkStatistics.downlink_RSSI = 120 + Radio.LastPacketRSSI;
    crsf.LinkStatistics.downlink_Link_quality = linkQuality;
    //crsf.LinkStatistics.downlink_Link_quality = Radio.currPWR;
    crsf.LinkStatistics.rf_Mode = 4 - ExpressLRS_currAirRate.enum_rate;

    crsf.TLMbattSensor.voltage = (Radio.RXdataBuffer[3] << 8) + Radio.RXdataBuffer[6];

    crsf.sendLinkStatisticsToTX();

    DEBUG_PRINT("[INFO] Received TLM packet with RSSI = ");
    DEBUG_PRINT(Radio.RXdataBuffer[2]);
    DEBUG_PRINT(" LQ = ");
    DEBUG_PRINT(Radio.RXdataBuffer[5]);
    DEBUG_PRINT(" RF mode = ");
    DEBUG_PRINTLN(4 - ExpressLRS_currAirRate.enum_rate);
  }
}

void ICACHE_RAM_ATTR CheckChannels5to8Change()
{ //check if channels 5 to 8 have new data (switch channels)
  for (int i = 4; i < 8; i++)
  {
    if (crsf.ChannelDataInPrev[i] != crsf.ChannelDataIn[i])
    {
      Channels5to8Changed = true;
    }
  }
}

void ICACHE_RAM_ATTR GenerateSyncPacketData()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + SYNC_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = FHSSgetCurrIndex();
  Radio.TXdataBuffer[2] = Radio.NonceTX;
  Radio.TXdataBuffer[3] = 0;
  Radio.TXdataBuffer[4] = UID[3];
  Radio.TXdataBuffer[5] = UID[4];
  Radio.TXdataBuffer[6] = UID[5];
}

void ICACHE_RAM_ATTR Generate4ChannelData_10bit()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + RC_DATA_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = ((CRSF_to_UINT10(crsf.ChannelDataIn[0]) & 0b1111111100) >> 2);
  Radio.TXdataBuffer[2] = ((CRSF_to_UINT10(crsf.ChannelDataIn[1]) & 0b1111111100) >> 2);
  Radio.TXdataBuffer[3] = ((CRSF_to_UINT10(crsf.ChannelDataIn[2]) & 0b1111111100) >> 2);
  Radio.TXdataBuffer[4] = ((CRSF_to_UINT10(crsf.ChannelDataIn[3]) & 0b1111111100) >> 2);
  Radio.TXdataBuffer[5] = ((CRSF_to_UINT10(crsf.ChannelDataIn[0]) & 0b0000000011) << 6) + ((CRSF_to_UINT10(crsf.ChannelDataIn[1]) & 0b0000000011) << 4) +
                          ((CRSF_to_UINT10(crsf.ChannelDataIn[2]) & 0b0000000011) << 2) + ((CRSF_to_UINT10(crsf.ChannelDataIn[3]) & 0b0000000011) << 0);
}

void ICACHE_RAM_ATTR Generate4ChannelData_11bit()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + RC_DATA_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = ((crsf.ChannelDataIn[0]) >> 3);
  Radio.TXdataBuffer[2] = ((crsf.ChannelDataIn[1]) >> 3);
  Radio.TXdataBuffer[3] = ((crsf.ChannelDataIn[2]) >> 3);
  Radio.TXdataBuffer[4] = ((crsf.ChannelDataIn[3]) >> 3);
  Radio.TXdataBuffer[5] = ((crsf.ChannelDataIn[0] & 0b00000111) << 5) + ((crsf.ChannelDataIn[1] & 0b111) << 2) + ((crsf.ChannelDataIn[2] & 0b110) >> 1);
  Radio.TXdataBuffer[6] = ((crsf.ChannelDataIn[2] & 0b001) << 7) + ((crsf.ChannelDataIn[3] & 0b111) << 4); // 4 bits left over for something else?
#ifdef One_Bit_Switches
  Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[4]) << 3;
  Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[5]) << 2;
  Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[6]) << 1;
  Radio.TXdataBuffer[6] += CRSF_to_BIT(crsf.ChannelDataIn[7]) << 0;
#endif
}

void ICACHE_RAM_ATTR GenerateSwitchChannelData()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + SWITCH_DATA_PACKET;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = ((CRSF_to_UINT10(crsf.ChannelDataIn[4]) & 0b1110000000) >> 2) + ((CRSF_to_UINT10(crsf.ChannelDataIn[5]) & 0b1110000000) >> 5) + ((CRSF_to_UINT10(crsf.ChannelDataIn[6]) & 0b1100000000) >> 8);
  Radio.TXdataBuffer[2] = (CRSF_to_UINT10(crsf.ChannelDataIn[6]) & 0b0010000000) + ((CRSF_to_UINT10(crsf.ChannelDataIn[7]) & 0b1110000000) >> 3);
  Radio.TXdataBuffer[3] = Radio.TXdataBuffer[1];
  Radio.TXdataBuffer[4] = Radio.TXdataBuffer[2];
  Radio.TXdataBuffer[5] = Radio.NonceTX;
  Radio.TXdataBuffer[6] = FHSSgetCurrIndex();
}

void ICACHE_RAM_ATTR SetRFLinkRate(expresslrs_mod_settings_s mode) // Set speed of RF link (hz)
{
  DEBUG_PRINT("[INFO] Setting RF link rate to mode ");
  DEBUG_PRINT(4 - mode.enum_rate);
  DEBUG_PRINT(" with a freq of ");
  DEBUG_PRINT(mode.rate);
  DEBUG_PRINTLN("Hz");

#ifdef PLATFORM_ESP32
  Radio.TimerInterval = mode.interval;
  Radio.UpdateTimerInterval();
#else
  hwTimer.updateInterval(mode.interval);
#endif
  Radio.Config(mode.bw, mode.sf, mode.cr, Radio.currFreq, Radio._syncWord);
  Radio.SetPreambleLength(mode.PreambleLen);
  ExpressLRS_prevAirRate = ExpressLRS_currAirRate;
  ExpressLRS_currAirRate = mode;
  crsf.RequestedRCpacketInterval = ExpressLRS_currAirRate.interval;
  
  updateRFConnectionState(disconnected);
  //R9DAC.resume();
}

uint8_t ICACHE_RAM_ATTR decRFLinkRate()
{
  DEBUG_PRINTLN("[INFO] Decreasing RF link rate...");
  if ((uint8_t)ExpressLRS_currAirRate.enum_rate < MaxRFrate)
  {
    SetRFLinkRate(ExpressLRS_AirRateConfig[(uint8_t)ExpressLRS_currAirRate.enum_rate + 1]);
  }
  return (uint8_t)ExpressLRS_currAirRate.enum_rate;
}

uint8_t ICACHE_RAM_ATTR incRFLinkRate()
{
  DEBUG_PRINTLN("[INFO] Increasing RF link rate...");
  if ((uint8_t)ExpressLRS_currAirRate.enum_rate > 0)
  {
    SetRFLinkRate(ExpressLRS_AirRateConfig[(uint8_t)ExpressLRS_currAirRate.enum_rate - 1]);
  }
  return (uint8_t)ExpressLRS_currAirRate.enum_rate;
}

void ICACHE_RAM_ATTR HandleFHSS()
{
  uint8_t modresult = (Radio.NonceTX) % ExpressLRS_currAirRate.FHSShopInterval;

  if (modresult == 0) // if it time to hop, do so.
  {
    // DEBUG_PRINT("[INFO] Hopping to next RF freq ");
    // DEBUG_PRINTLN(SX127xDriver::currFreq);
    Radio.SetFrequency(FHSSgetNextFreq());
  }
}

void ICACHE_RAM_ATTR HandleTLM()
{
  if (ExpressLRS_currAirRate.TLMinterval > 0)
  {
    uint8_t modresult = (Radio.NonceTX) % TLMratioEnumToValue(ExpressLRS_currAirRate.TLMinterval);
    if (modresult != 0) // wait for tlm response because it's time
    {
      return;
    }

#ifdef TARGET_R9M_TX
    //R9DAC.standby(); //takes too long
    digitalWrite(GPIO_PIN_RFswitch_CONTROL, 1);
    digitalWrite(GPIO_PIN_RFamp_APC1, 0);
#endif

    Radio.RXnb();
    WaitRXresponse = true;
    // DEBUG_PRINTLN("[INFO] Setting RF to RXnb mode for TLM response");
  }
}

void ICACHE_RAM_ATTR SendRCdataToRF()
{

#ifdef FEATURE_OPENTX_SYNC
  crsf.JustSentRFpacket(); // tells the crsf that we want to send data now - this allows opentx packet syncing
#endif

  /////// This Part Handles the Telemetry Response ///////
  if (ExpressLRS_currAirRate.TLMinterval > 0)
  {
    uint8_t modresult = (Radio.NonceTX) % TLMratioEnumToValue(ExpressLRS_currAirRate.TLMinterval);
    if (modresult == 0)
    { // wait for tlm response
      if (WaitRXresponse == true)
      {
        WaitRXresponse = false;
        return;
      }
      else
      {
        Radio.NonceTX++;
      }
    }
  }

  uint32_t SyncInterval;

  if (connectionState == connected)
  {
    SyncInterval = SYNC_PACKET_SEND_INTERVAL_RX_CONN;
  }
  else
  {
    SyncInterval = SYNC_PACKET_SEND_INTERVAL_RX_LOST;
  }

  //if (((millis() > (SyncPacketLastSent + SyncInterval)) && (Radio.currFreq == GetInitialFreq())) || ChangeAirRateRequested) //only send sync when its time and only on channel 0;
  if ((millis() > (SyncPacketLastSent + SyncInterval)) && (Radio.currFreq == GetInitialFreq()))
  {

    GenerateSyncPacketData();
    SyncPacketLastSent = millis();
    ChangeAirRateSentUpdate = true;
    DEBUG_PRINT("[INFO] Sent RF sync packet at ");
    DEBUG_PRINTLN(Radio.currFreq);
  }
  else
  {
    if ((millis() > (SWITCH_PACKET_SEND_INTERVAL + SwitchPacketLastSent)) || Channels5to8Changed)
    {
      Channels5to8Changed = false;
      GenerateSwitchChannelData();
      SwitchPacketLastSent = millis();
    }
    else // else we just have regular channel data which we send as 8 + 2 bits
    {
      Generate4ChannelData_11bit();
    }
  }

  ///// Next, Calculate the CRC and put it into the buffer /////
  uint8_t crc = CalcCRC(Radio.TXdataBuffer, 7) + CRCCaesarCipher;
  Radio.TXdataBuffer[7] = crc;
#ifdef TARGET_R9M_TX
  //R9DAC.resume(); takes too long
  digitalWrite(GPIO_PIN_RFswitch_CONTROL, 0);
  digitalWrite(GPIO_PIN_RFamp_APC1, 1);
#endif

  Radio.TXnb(Radio.TXdataBuffer, 8);

  if (ChangeAirRateRequested)
  {
    ChangeAirRateSentUpdate = true;
  }
}

void ICACHE_RAM_ATTR ParamUpdateReq()
{
  UpdateParamReq = true;
}

void ICACHE_RAM_ATTR HandleUpdateParameter()
{
  if (!UpdateParamReq)
  {
    return;
  }

  switch (crsf.ParameterUpdateData[0])
  {
  case 0: // send all params
    DEBUG_PRINTLN("[INFO] LUA: Got request to send all params");
    crsf.sendLUAresponse((ExpressLRS_currAirRate.enum_rate + 2), ExpressLRS_currAirRate.TLMinterval + 1, 7, 1);
    break;
  case 1:
    DEBUG_PRINTLN("[INFO] LUA: Got request to update RF link rate");
    // if (ExpressLRS_currAirRate.enum_rate != (expresslrs_RFrates_e)crsf.ParameterUpdateData[1])
    // {
    //   SetRFLinkRate(ExpressLRS_AirRateConfig[crsf.ParameterUpdateData[1]]);
    // }
    //crsf.sendLUAresponse(0x01, (uint8_t)random(1, 5));
    if (crsf.ParameterUpdateData[1] == 0)
    {
      uint8_t newRate = decRFLinkRate();
    }
    else if (crsf.ParameterUpdateData[1] == 1)
    {
      uint8_t newRate = incRFLinkRate();
    }
    crsf.sendLUAresponse((ExpressLRS_currAirRate.enum_rate + 2), ExpressLRS_currAirRate.TLMinterval + 1, 7, 1);
    break;

  case 2:

    break;
  case 3:
    DEBUG_PRINTLN("[INFO] LUA: Got request to update RF power");
    switch (crsf.ParameterUpdateData[1])
    {
    case 0:
      Radio.maxPWR = 0b1111;
      Radio.SetOutputPower(0b1111); // 500 mW
      DEBUG_PRINTLN("[INFO] Setpower 500 mW");
      break;

    case 1:
      Radio.maxPWR = 0b1000;
      Radio.SetOutputPower(0b1000);
      DEBUG_PRINTLN("[INFO] Setpower 200 mW");
      break;

    case 2:
      Radio.maxPWR = 0b1000;
      Radio.SetOutputPower(0b1000);
      DEBUG_PRINTLN("[INFO] Setpower 100 mW");
      break;

    case 3:
      Radio.maxPWR = 0b0101;
      Radio.SetOutputPower(0b0101);
      DEBUG_PRINTLN("[INFO] Setpower 50 mW");
      break;

    case 4:
      Radio.maxPWR = 0b0010;
      Radio.SetOutputPower(0b0010);
      DEBUG_PRINTLN("[INFO] Setpower 25 mW");
      break;

    case 5:
      Radio.maxPWR = 0b0000;
      Radio.SetOutputPower(0b0000);
      DEBUG_PRINTLN("[INFO] Setpower Pit");
      break;

    default:
      break;
    }

    break;
  case 4:

    break;

  default:
    break;
  }

  UpdateParamReq = false;
}

void DetectOtherRadios()
{
  Radio.SetFrequency(GetInitialFreq());
  //Radio.RXsingle();

  // if (Radio.RXsingle(RXdata, 7, 2 * (RF_RATE_50HZ.interval / 1000)) == ERR_NONE)
  // {
  //   DEBUG_PRINTLN("got fastsync resp 1");
  //   break;
  // }
}

void setup()
{
  // Setup debug serial ports first
#ifdef PLATFORM_ESP32
  Serial.begin(400000);
  DEBUG_PRINTLN("[INFO] Booting ESP32 TX...");
#endif

#ifdef TARGET_R9M_TX
  #ifdef USE_USART2_FOR_DEBUG
    HardwareSerial(USART2);
    Serial.setTx(GPIO_PIN_DEBUG_TX);
    Serial.setRx(GPIO_PIN_DEBUG_RX);
    Serial.begin(400000);
    DEBUG_PRINTLN("[INFO] Using USART2 for debug");
  #else
    HardwareSerial(USART1);
    Serial.setTx(PA9);
    Serial.setRx(PA10);
    Serial.begin(250000);
    DEBUG_PRINTLN("[INFO] Using USART1 for debug");
  #endif
  DEBUG_PRINTLN("[INFO] Booting R9M TX...");
#endif

#ifdef TARGET_EXPRESSLRS_PCB_TX_V3_LEGACY
  pinMode(RC_SIGNAL_PULLDOWN, INPUT_PULLDOWN);
  pinMode(GPIO_PIN_BUTTON, INPUT_PULLUP);
  DEBUG_PRINTLN("[INFO] Using legacy pinModes");
#endif

#ifdef PLATFORM_ESP32
  crsf.connected = &Radio.StartTimerTask;
  crsf.disconnected = &Radio.StopTimerTask;
  crsf.RecvParameterUpdate = &ParamUpdateReq;
  Radio.TimerDoneCallback = &TimerExpired;
#endif

#ifdef TARGET_R9M_TX
  // Annoying startup beeps
  pinMode(GPIO_PIN_BUZZER, OUTPUT);
  const int beepFreq[] = {659, 659, 659, 523, 659, 783, 392};
  const int beepDurations[] = {150, 300, 300, 100, 300, 550, 575};

  for (int i = 0; i < 7; i++)
  {
    tone(GPIO_PIN_BUZZER, beepFreq[i], beepDurations[i]);
    delay(beepDurations[i]);
    noTone(GPIO_PIN_BUZZER);
  }

  pinMode(GPIO_PIN_LED_GREEN, OUTPUT);
  pinMode(GPIO_PIN_LED_RED, OUTPUT);

  digitalWrite(GPIO_PIN_LED_GREEN, HIGH);

  pinMode(GPIO_PIN_RFswitch_CONTROL, OUTPUT);
  pinMode(GPIO_PIN_RFamp_APC1, OUTPUT);
  digitalWrite(GPIO_PIN_RFamp_APC1, HIGH);

  R9DAC.init(GPIO_PIN_SDA, GPIO_PIN_SCL, 0b0001100); // used to control ADC which sets PA output
  DEBUG_PRINTLN("[INFO] Setting RF output power to 50mW");
  R9DAC.setPower(R9_PWR_50mW);

  button.init(GPIO_PIN_BUTTON, true); // r9 tx appears to be active high

  crsf.connected = &hwTimer.init; // it will auto init when it detects UART connection
  crsf.disconnected = &hwTimer.stop;
  crsf.RecvParameterUpdate = &ParamUpdateReq;
  hwTimer.callbackTock = &TimerExpired;
#endif

#ifdef TARGET_EXPRESSLRS_PCB_TX_V3
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  strip.Begin();

  // Get base mac address
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  // Print base mac address
  // This should be copied to common.h and is used to generate a unique hop sequence, DeviceAddr, and CRC.
  // UID[0..2] are OUI (organisationally unique identifier) and are not ESP32 unique.  Do not use!
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("[INFO] Copy the below line into common.h.");
  DEBUG_PRINT("uint8_t UID[6] = {");
  DEBUG_PRINT(baseMac[0]);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(baseMac[1]);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(baseMac[2]);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(baseMac[3]);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(baseMac[4]);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(baseMac[5]);
  DEBUG_PRINTLN("};");
  DEBUG_PRINTLN("");
#endif

  FHSSrandomiseFHSSsequence();

#if defined Regulatory_Domain_AU_915 || defined Regulatory_Domain_EU_868
  #ifdef Regulatory_Domain_AU_915
    DEBUG_PRINTLN("[INFO] Setting 915MHz Mode");
  #else
    DEBUG_PRINTLN("[INFO] Setting 868MHz Mode");
  #endif
  Radio.RFmodule = RFMOD_SX1276; //define radio module here
#ifdef TARGET_100mW_MODULE
  DEBUG_PRINTLN("[INFO] Setting RF output power to 100mW");
  Radio.SetOutputPower(0b1111); // 20dbm = 100mW
#else                           // Below output power settings are for 1W modules
  // Radio.SetOutputPower(0b0000); // 15dbm = 32mW
  // Radio.SetOutputPower(0b0001); // 18dbm = 40mW
  // Radio.SetOutputPower(0b0101); // 20dbm = 100mW
  Radio.SetOutputPower(0b1000); // 23dbm = 200mW
                                // Radio.SetOutputPower(0b1100); // 27dbm = 500mW
                                // Radio.SetOutputPower(0b1111); // 30dbm = 1000mW
#endif
#elif defined Regulatory_Domain_AU_433 || defined Regulatory_Domain_EU_433
  DEBUG_PRINTLN("[INFO] Setting 433MHz Mode");
  Radio.RFmodule = RFMOD_SX1278; //define radio module here
  DEBUG_PRINTLN("[INFO] Setting RF output power to 100mW");
  Radio.SetOutputPower(0b1111);
#endif

  Radio.SetFrequency(GetInitialFreq()); //set frequency first or an error will occur!!!

  Radio.RXdoneCallback1 = &ProcessTLMpacket;

  Radio.TXdoneCallback1 = &HandleFHSS;
  Radio.TXdoneCallback2 = &HandleTLM;
  Radio.TXdoneCallback3 = &HandleUpdateParameter;
  //Radio.TXdoneCallback4 = &NULL;

#ifndef One_Bit_Switches
  DEBUG_PRINTLN("[INFO] Using 1-bit switches");
  crsf.RCdataCallback1 = &CheckChannels5to8Change;
#endif

  Radio.Begin();
  crsf.Begin();

  SetRFLinkRate(RF_RATE_200HZ);
  DEBUG_PRINTLN("[INFO] Setup complete - ExpressLRS TX Module Booted");
}

void loop()
{
#ifdef FEATURE_OPENTX_SYNC
  //DEBUG_PRINTLN(crsf.OpenTXsyncOffset);
#endif

  //updateLEDs(connectionState == connected, ExpressLRS_currAirRate.TLMinterval);

  if (millis() > (RX_CONNECTION_LOST_TIMEOUT + LastTLMpacketRecvMillis))
  {
    if (connectionState == connected)
    {
      DEBUG_PRINT("[ERROR] No RF response from RX in ");
      DEBUG_PRINT(RX_CONNECTION_LOST_TIMEOUT);
      DEBUG_PRINTLN("ms");
    }
    updateRFConnectionState(disconnected);
#ifdef TARGET_R9M_TX
    digitalWrite(GPIO_PIN_LED_RED, LOW);
#endif
  }
  else
  {
    updateRFConnectionState(connected);
#ifdef TARGET_R9M_TX
    digitalWrite(GPIO_PIN_LED_RED, HIGH);
#endif
  }

  float targetFrameRate = (ExpressLRS_currAirRate.rate * (1.0 / TLMratioEnumToValue(ExpressLRS_currAirRate.TLMinterval)));
  PacketRateLastChecked = millis();
  PacketRate = (float)packetCounteRX_TX / (float)(PACKET_RATE_INTERVAL);
  linkQuality = int((((float)PacketRate / (float)targetFrameRate) * 100000.0));

  if (linkQuality > 99)
  {
    linkQuality = 99;
  }
  packetCounteRX_TX = 0;

#ifdef TARGET_R9M_TX
  crsf.STM32handleUARTin();
  crsf.sendSyncPacketToTX();
  crsf.STM32wdtUART();
  button.handle();
#endif
}

void ICACHE_RAM_ATTR TimerExpired()
{
  SendRCdataToRF();
}

void updateRFConnectionState(connectionState_e newState)
{
    if (newState != connectionState)
    {
      // Connection state has changed. Update the state
      connectionStatePrev = connectionState;
      connectionState = newState;

      // Log the new connection state
      if (connectionState == connected)
      {
        DEBUG_PRINTLN("[INFO] RF connection established");
      }
      else
      {
        DEBUG_PRINTLN("[ERROR] RF connection lost");
      }
    }
}
