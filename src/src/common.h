#pragma once

#include "FHSS.h"
#include "LoRaRadioLib.h"
#include <Arduino.h>

// define frequnecy band of operation
#define Regulatory_Domain_AU_915
// #define Regulatory_Domain_AU_433

// Wifi starts if no connection is found between 10 and 11 seconds after boot
#define Auto_WiFi_On_Boot

#define One_Bit_Switches

extern uint8_t UID[6];
extern uint8_t CRCCaesarCipher;
extern uint8_t DeviceAddr;

typedef enum
{
    TLM_RATIO_NO_TLM = 0,
    TLM_RATIO_1_128 = 1,
    TLM_RATIO_1_64 = 2,
    TLM_RATIO_1_32 = 3,
    TLM_RATIO_1_16 = 4,
    TLM_RATIO_1_8 = 5,
    TLM_RATIO_1_4 = 6,
    TLM_RATIO_1_2 = 7

} expresslrs_tlm_ratio_e;

typedef enum
{
    connected = 2,
    tentative = 1,
    disconnected = 0
} connectionState_e;

extern connectionState_e connectionState;
extern connectionState_e connectionStatePrev;

typedef enum
{
    RF_DOWNLINK_INFO = 0,
    RF_UPLINK_INFO = 1,
    RF_AIRMODE_PARAMETERS = 2
} expresslrs_tlm_header_e;

typedef enum
{
    RATE_4HZ = 4,
    RATE_25HZ = 3,
    RATE_50HZ = 2,
    RATE_100HZ = 1,
    RATE_200HZ = 0,
} expresslrs_RFrates_e; // Max value of 16 since only 4 bits have been assigned in the sync package.

#define MaxRFrate 2

typedef struct expresslrs_mod_settings_s
{
    Bandwidth bw;
    SpreadingFactor sf;
    CodingRate cr;
    int32_t sensitivity;                //expected RF sensitivity based on
    uint32_t interval;                  //interval in us seconds that corresponds to that frequnecy
    uint8_t rate;                       // rate in hz
    expresslrs_tlm_ratio_e TLMinterval; // every X packets is a response TLM packet, should be a power of 2
    uint8_t FHSShopInterval;            // every X packets we hope to a new frequnecy. Max value of 16 since only 4 bits have been assigned in the sync package.
    uint8_t PreambleLen;
    expresslrs_RFrates_e enum_rate; // Max value of 16 since only 4 bits have been assigned in the sync package.
    uint16_t RFmodeCycleAddtionalTime;
    uint16_t RFmodeCycleInterval;
} expresslrs_mod_settings_t;

extern expresslrs_mod_settings_s RF_RATE_200HZ;
extern expresslrs_mod_settings_s RF_RATE_100HZ;
extern expresslrs_mod_settings_s RF_RATE_50HZ;
extern expresslrs_mod_settings_s RF_RATE_25HZ;
extern expresslrs_mod_settings_s RF_RATE_4HZ;

extern const expresslrs_mod_settings_s ExpressLRS_AirRateConfig[5];

extern expresslrs_mod_settings_s ExpressLRS_nextAirRate;
extern expresslrs_mod_settings_s ExpressLRS_currAirRate;
extern expresslrs_mod_settings_s ExpressLRS_prevAirRate;

#define MaxPower100mW_Module 20
#define MaxPower1000mW_Module 30
#define RF_Gain 10

extern int8_t ExpressLRS_currPower;
extern int8_t ExpressLRS_prevPower;

int16_t MeasureNoiseFloor();
int16_t MeasureRSSI(int FHSSindex);
uint8_t TLMratioEnumToValue(expresslrs_tlm_ratio_e enumval);

// expresslrs packet header types
// 00 -> standard 4 channel data packet
// 01 -> switch data packet
// 11 -> tlm packet
// 10 -> sync packet with hop data
#define RC_DATA_PACKET      0b00
#define SWITCH_DATA_PACKET  0b01
#define TLM_PACKET          0b11
#define SYNC_PACKET         0b10

///// BINDING /////
#define BINDING_CIPHER 100
#define BINDING_ADDR   50
#define PREVENT_BIND_WHEN_CONNECTED true // Disable binding when connected to TX
#define USE_FLASH_FOR_UID           true // Set to false to use hardcoded Unique ID
