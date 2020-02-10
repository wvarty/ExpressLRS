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

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

String DebugOutput;

/// define some libs to use ///
SX127xDriver Radio;
CRSF crsf;

bool InBindingMode = false;
void EnterBindingMode();
void ExitBindingMode();
void CancelBindingMode();
void PrintMac();
void UpdateConnectionState(connectionState_e state);

//// Switch Data Handling ///////
uint8_t SwitchPacketsCounter = 0;             //not used for the moment
uint32_t SwitchPacketSendInterval = 200;      //not used, delete when able to
uint32_t SyncPacketSendIntervalRXlost = 250;  //how often to send the switch data packet (ms) when there is no response from RX
uint32_t SyncPacketSendIntervalRXconn = 1500; //how often to send the switch data packet (ms) when there we have a connection
uint32_t SwitchPacketLastSent = 0;            //time in ms when the last switch data packet was sent

////////////SYNC PACKET/////////
uint32_t SyncPacketLastSent = 0;

uint32_t LastTLMpacketRecvMillis = 0;
uint32_t RXconnectionLostTimeout = 1500; //After 1500ms of no TLM response consider that slave has lost connection
int packetCounteRX_TX = 0;
uint32_t PacketRateLastChecked = 0;
uint32_t PacketRateInterval = 500;
float PacketRate = 0.0;
uint8_t linkQuality = 0;

/// Variables for Sync Behaviour ////
uint32_t RFmodeLastCycled = 0;
uint32_t RFmodeCycleInterval = 1000;
uint32_t SyncPacketAddtionalTime = 1500; //After we have a tentative sync we wait this long in addtion before jumping to different RF mode again.
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

void ICACHE_RAM_ATTR ProcessTLMpacket()
{
  uint8_t calculatedCRC = CalcCRC(Radio.RXdataBuffer, 7) + CRCCaesarCipher;
  uint8_t inCRC = Radio.RXdataBuffer[7];
  uint8_t type = Radio.RXdataBuffer[0] & 0b11;
  uint8_t packetAddr = (Radio.RXdataBuffer[0] & 0b11111100) >> 2;
  uint8_t TLMheader = Radio.RXdataBuffer[1];

  //DEBUG_PRINTLN("TLMpacket0");

  if (packetAddr != DeviceAddr) {
    DEBUG_PRINTLN("TLM dev addr error");
  }

  if (inCRC != calculatedCRC) {
    DEBUG_PRINTLN("TLM crc error");
  }

  packetCounteRX_TX++;
  
  if (type != TLM_PACKET) {
    DEBUG_PRINTLN("TLM type error");
    DEBUG_PRINTLN(type);
  }

  //DEBUG_PRINTLN("TLMpacket1");
  //DEBUG_PRINTLN(type);
  UpdateConnectionState(connected);
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
    crsf.sendLinkStatisticsToTX();
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
  PacketHeaderAddr = (DeviceAddr << 2) + 0b10;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = FHSSgetCurrIndex();
  Radio.TXdataBuffer[2] = Radio.NonceTX;
  Radio.TXdataBuffer[3] = 0;
  Radio.TXdataBuffer[4] = TxBaseMac[3];
  Radio.TXdataBuffer[5] = TxBaseMac[4];
  Radio.TXdataBuffer[6] = TxBaseMac[5];
}

void ICACHE_RAM_ATTR Generate4ChannelData_10bit()
{
  uint8_t PacketHeaderAddr;
  PacketHeaderAddr = (DeviceAddr << 2) + 0b00;
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
  PacketHeaderAddr = (DeviceAddr << 2) + 0b00;
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
  PacketHeaderAddr = (DeviceAddr << 2) + 0b01;
  Radio.TXdataBuffer[0] = PacketHeaderAddr;
  Radio.TXdataBuffer[1] = ((CRSF_to_UINT10(crsf.ChannelDataIn[4]) & 0b1110000000) >> 2) + ((CRSF_to_UINT10(crsf.ChannelDataIn[5]) & 0b1110000000) >> 5) + ((CRSF_to_UINT10(crsf.ChannelDataIn[6]) & 0b1100000000) >> 8);
  Radio.TXdataBuffer[2] = (CRSF_to_UINT10(crsf.ChannelDataIn[6]) & 0b0010000000) + ((CRSF_to_UINT10(crsf.ChannelDataIn[7]) & 0b1110000000) >> 3);
  Radio.TXdataBuffer[3] = Radio.TXdataBuffer[1];
  Radio.TXdataBuffer[4] = Radio.TXdataBuffer[2];
  Radio.TXdataBuffer[5] = Radio.NonceTX;
  Radio.TXdataBuffer[6] = FHSSgetCurrIndex();
}

void SetRFLinkRate(expresslrs_mod_settings_s mode) // Set speed of RF link (hz)
{
  Radio.TimerInterval = mode.interval;
  Radio.UpdateTimerInterval();
  Radio.Config(mode.bw, mode.sf, mode.cr, Radio.currFreq, Radio._syncWord);
  Radio.SetPreambleLength(mode.PreambleLen);
  ExpressLRS_prevAirRate = ExpressLRS_currAirRate;
  ExpressLRS_currAirRate = mode;
  crsf.RequestedRCpacketInterval = ExpressLRS_currAirRate.interval;
  UpdateConnectionState(disconnected);
}

void ICACHE_RAM_ATTR HandleFHSS()
{
  if (FreqLocked) {
    return;
  }
  
  uint8_t modresult = (Radio.NonceTX) % ExpressLRS_currAirRate.FHSShopInterval;

  if (modresult == 0) // if it time to hop, do so.
  {
    Radio.SetFrequency(FHSSgetNextFreq());
  }
}

void ICACHE_RAM_ATTR HandleTLM()
{
  if (ExpressLRS_currAirRate.TLMinterval > 0)
  {
    uint8_t modresult = (Radio.NonceTX) % TLMratioEnumToValue(ExpressLRS_currAirRate.TLMinterval);

    if (modresult == 0) // wait for tlm response because it's time
    {
      Radio.RXnb();
      WaitRXresponse = true;
    }
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
    SyncInterval = SyncPacketSendIntervalRXconn;
  }
  else
  {
    SyncInterval = SyncPacketSendIntervalRXlost;
  }

  //if (((millis() > (SyncPacketLastSent + SyncInterval)) && (Radio.currFreq == GetInitialFreq())) || ChangeAirRateRequested) //only send sync when its time and only on channel 0;+
  if ((millis() > (SyncPacketLastSent + SyncInterval)) && (Radio.currFreq == GetInitialFreq() || InBindingMode))
  {

    GenerateSyncPacketData();
    SyncPacketLastSent = millis();
    ChangeAirRateSentUpdate = true;
    //DEBUG_PRINTLN("sync");
    //DEBUG_PRINTLN(Radio.currFreq);
  }
  else
  {
    if ((millis() > (SwitchPacketSendInterval + SwitchPacketLastSent)) || Channels5to8Changed)
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

  if (UpdateParamReq == true)
  {
    switch (crsf.ParameterUpdateData[0])
    {
    case 1:
      if (ExpressLRS_currAirRate.enum_rate != (expresslrs_RFrates_e)crsf.ParameterUpdateData[1])
      {
        SetRFLinkRate(ExpressLRS_AirRateConfig[crsf.ParameterUpdateData[1]]);
      }
      break;

    case 2:

      break;
    case 3:

      switch (crsf.ParameterUpdateData[1])
      {
      case 0:
        Radio.maxPWR = 0b1111;
        Radio.SetOutputPower(0b1111); // 500 mW
        Serial.println("Setpower 500 mW");
        break;

      case 1:
        Radio.maxPWR = 0b1000;
        Radio.SetOutputPower(0b1111);
        Serial.println("Setpower 200 mW");
        break;

      case 2:
        Radio.maxPWR = 0b1000;
        Radio.SetOutputPower(0b1000);
        Serial.println("Setpower 100 mW");
        break;

      case 3:
        Radio.maxPWR = 0b0101;
        Radio.SetOutputPower(0b0101);
        Serial.println("Setpower 50 mW");
        break;

      case 4:
        Radio.maxPWR = 0b0010;
        Radio.SetOutputPower(0b0010);
        Serial.println("Setpower 25 mW");
        break;

      case 5:
        Radio.maxPWR = 0b0000;
        Radio.SetOutputPower(0b0000);
        Serial.println("Setpower Pit");
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
  }

  UpdateParamReq = false;
}

// void ICACHE_RAM_ATTR IncreasePower()
// {
//   if (Radio.currPWR < Radio.maxPWR)
//   {
//     Radio.SetOutputPower(Radio.currPWR + 1);
//   }
// }

// void ICACHE_RAM_ATTR DecreasePower()
// {
//   if (Radio.currPWR > 0)
//   {
//     Radio.SetOutputPower(Radio.currPWR - 1);
//   }
// }

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
#ifdef LEGACY_HARDWARE
  pinMode(4, INPUT_PULLDOWN);
#endif
  pinMode(36, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("ExpressLRS TX Module Booted...");
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  strip.Begin();
  for(int n = 0; n < 3; n++) strip.SetPixelColor(n, RgbColor(255, 0, 0));
  strip.Show();

  // Get base mac address
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);

  // Print base mac address
  // This should be copied to common.h and is used to generate a unique hop sequence, DeviceAddr, and CRC.
  // TxBaseMac[0..2] are OUI (organisationally unique identifier) and are not ESP32 unique.  Do not use!
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("Copy the below line into common.h.");
  DEBUG_PRINT("uint8_t TxBaseMac[6] = {");
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

  FHSSrandomiseFHSSsequence();

#ifdef Regulatory_Domain_AU_915
  DEBUG_PRINTLN("Setting 915MHz Mode");
  Radio.RFmodule = RFMOD_SX1276; //define radio module here
#ifdef TARGET_100mW_MODULE
  Radio.SetOutputPower(0b1111); // 20dbm = 100mW
#else                           // Below output power settings are for 1W modules
  // Radio.SetOutputPower(0b0000); // 15dbm = 32mW
  // Radio.SetOutputPower(0b0001); // 18dbm = 40mW
  // Radio.SetOutputPower(0b0101); // 20dbm = 100mW
  Radio.SetOutputPower(0b1000); // 23dbm = 200mW
                                // Radio.SetOutputPower(0b1100); // 27dbm = 500mW
                                // Radio.SetOutputPower(0b1111); // 30dbm = 1000mW
#endif
#elif defined Regulatory_Domain_AU_433
  DEBUG_PRINTLN("Setting 433MHz Mode");
  Radio.RFmodule = RFMOD_SX1278; //define radio module here
  Radio.SetOutputPower(0b1111);
#endif

  Radio.SetFrequency(GetInitialFreq()); //set frequency first or an error will occur!!!

  Radio.RXdoneCallback1 = &ProcessTLMpacket;

  Radio.TXdoneCallback1 = &HandleFHSS;
  Radio.TXdoneCallback2 = &HandleTLM;
  Radio.TXdoneCallback3 = &HandleUpdateParameter;
  //Radio.TXdoneCallback4 = &NULL;

  Radio.TimerDoneCallback = &SendRCdataToRF;

#ifndef One_Bit_Switches
  crsf.RCdataCallback1 = &CheckChannels5to8Change;
#endif
  crsf.connected = &Radio.StartTimerTask;
  crsf.disconnected = &Radio.StopTimerTask;
  crsf.RecvParameterUpdate = &ParamUpdateReq;

  Radio.Begin();
  SetRFLinkRate(RF_RATE_200HZ);
  // SetRFLinkRate(RF_RATE_100HZ);
  crsf.Begin();
}

void loop()
{

  delay(100);

  // if (digitalRead(4) == 0)
  // {
  //   Serial.println("Switch Pressed!");
  // }

  // if (digitalRead(36) == 0)
  // {
  //   Serial.println("Switch Pressed!");
  // }

#ifdef FEATURE_OPENTX_SYNC
  DEBUG_PRINTLN(crsf.OpenTXsyncOffset);
#endif

  updateLEDs(isRXconnected, ExpressLRS_currAirRate.TLMinterval);

  if (millis() > (RXconnectionLostTimeout + LastTLMpacketRecvMillis))
  {
    UpdateConnectionState(disconnected);
  }
  else
  {
    UpdateConnectionState(connected);
  }

  //if (millis() > (PacketRateLastChecked + PacketRateInterval)//just some debug data
  //{
  // if (connectionState == connected)
  // {
  //   if ((Radio.RXdataBuffer[2] < 30 || Radio.RXdataBuffer[4] < 10))
  //   {
  //     IncreasePower();
  //   }
  //   if (Radio.RXdataBuffer[2] > 60 || Radio.RXdataBuffer[4] > 40)
  //   {
  //     DecreasePower();
  //   }
  //   crsf.sendLinkStatisticsToTX();
  // }

  float targetFrameRate = (ExpressLRS_currAirRate.rate * (1.0 / TLMratioEnumToValue(ExpressLRS_currAirRate.TLMinterval)));
  PacketRateLastChecked = millis();
  PacketRate = (float)packetCounteRX_TX / (float)(PacketRateInterval);
  linkQuality = int((((float)PacketRate / (float)targetFrameRate) * 100000.0));

  if (linkQuality > 99)
  {
    linkQuality = 99;
  }
  packetCounteRX_TX = 0;
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
    if ((PREVENT_BIND_WHEN_CONNECTED && connectionState == connected) || InBindingMode) {
        // Don't enter binding if:
        // - we're already connected
        // - we're already binding
        return;
    }

    // Use binding cipher and addr
    CRCCaesarCipher = BindingCipher;
    DeviceAddr = BindingAddr;

    // Start attempting to bind
    // Lock the RF rate and freq to the base freq while binding
    SetRFLinkRate(RF_RATE_200HZ);
    Radio.SetFrequency(919100000);
    FreqLocked = true;

    InBindingMode = true;
    UpdateConnectionState(disconnected);

    DEBUG_PRINTLN("=== Entered binding mode ===");
    PrintMac();

    for(int n = 0; n < 3; n++) strip.SetPixelColor(n, RgbColor(0, 0, 255));
    strip.Show();
}

void ExitBindingMode()
{
  if (!InBindingMode) {
    // Not in binding mode
    return;
  }

  CRCCaesarCipher = TxBaseMac[4];
  DeviceAddr = TxBaseMac[5] & 0b111111;

  // Revert to original packet rate
  // and go to initial freq
  FreqLocked = false;
  SetRFLinkRate(RF_RATE_200HZ);
  Radio.SetFrequency(GetInitialFreq());

  InBindingMode = false;
  UpdateConnectionState(disconnected);

  DEBUG_PRINTLN("=== Binding successful ===");
  PrintMac();

  for(int n = 0; n < 3; n++) strip.SetPixelColor(n, RgbColor(0, 255, 0));
  strip.Show();
}

void CancelBindingMode()
{
  if (!InBindingMode) {
    // Not in binding mode
    return;
  }

  // Revert to original packet rate
  // and go to initial freq
  SetRFLinkRate(ExpressLRS_prevAirRate);
  Radio.SetFrequency(GetInitialFreq());

  // Revert to original cipher and addr
  CRCCaesarCipher = TxBaseMac[4];
  DeviceAddr = TxBaseMac[5] & 0b111111;

  // Binding cancelled
  // Go to a disconnected state
  InBindingMode = false;
  UpdateConnectionState(disconnected);

  DEBUG_PRINTLN("=== Binding mode cancelled ===");
  PrintMac();

  for(int n = 0; n < 3; n++) strip.SetPixelColor(n, RgbColor(255, 0, 0));
  strip.Show();
}

void UpdateConnectionState(connectionState_e state)
{
    connectionStatePrev = connectionState;
    connectionState = state;
}
