#include "LoRa_SX127x.h"

// define frequnecy band of operation
#define Regulatory_Domain_AU_915
// #define Regulatory_Domain_AU_433

// Wifi starts if no connection is found between 10 and 11 seconds after boot
#define Auto_WiFi_On_Boot

#define One_Bit_Switches

//uint8_t TxBaseMac[6] = {48, 174, 164, 200, 100, 50};
//uint8_t TxBaseMac[6] = {180, 230, 45, 152, 126, 65}; //sandro mac
uint8_t TxBaseMac[6] = {180, 230, 45, 152, 125, 173}; // Wez's MAC

// uint8_t TxBaseMac[6] = {0, 0, 0, 0, 0, 0};

uint8_t CRCCaesarCipher = TxBaseMac[4];
uint8_t DeviceAddr = TxBaseMac[5] & 0b111111; // temporarily based on mac until listen before assigning method merged

uint8_t BindingCipher = 100;
uint8_t BindingAddr = 50 & 0b111111;

bool FreqLocked = false;

typedef enum
{
    connected = 2,
    tentative = 1,
    disconnected = 0
} connectionState_e;

connectionState_e connectionState = disconnected;
connectionState_e connectionStatePrev = disconnected;

typedef enum
{
    RF_DOWNLINK_INFO,
    RF_UPLINK_INFO
} expresslrs_tlm_header_e;

typedef enum
{
    RATE_200HZ = 0,
    RATE_100HZ = 1,
    RATE_50HZ = 2,
    RATE_25HZ = 3,
    RATE_4HZ = 4

} expresslrs_RFrates_e; // Max value of 16 since only 4 bits have been assigned in the sync package.

typedef struct expresslrs_mod_settings_s
{
    Bandwidth bw;
    SpreadingFactor sf;
    CodingRate cr;
    uint32_t interval;       //interval in us seconds that corresponds to that frequnecy
    uint8_t rate;            // rate in hz
    uint8_t TLMinterval;     // every X packets is a response TLM packet, should be a power of 2
    uint8_t FHSShopInterval; // every X packets we hope to a new frequnecy. Max value of 16 since only 4 bits have been assigned in the sync package.
    uint8_t PreambleLen;
    expresslrs_RFrates_e enum_rate; // Max value of 16 since only 4 bits have been assigned in the sync package.

} expresslrs_mod_settings_t;

expresslrs_mod_settings_s RF_RATE_200HZ = {BW_500_00_KHZ, SF_6, CR_4_5, 5000, 200, 64, 4, 8, RATE_200HZ};
expresslrs_mod_settings_s RF_RATE_100HZ = {BW_500_00_KHZ, SF_7, CR_4_7, 10000, 100, 32, 4, 8, RATE_100HZ};
expresslrs_mod_settings_s RF_RATE_50HZ = {BW_500_00_KHZ, SF_8, CR_4_7, 20000, 50, 16, 2, 10, RATE_50HZ};
expresslrs_mod_settings_s RF_RATE_25HZ = {BW_250_00_KHZ, SF_8, CR_4_7, 40000, 25, 0, 2, 8, RATE_25HZ};
expresslrs_mod_settings_s RF_RATE_4HZ = {BW_250_00_KHZ, SF_11, CR_4_5, 250000, 4, 0, 2, 8, RATE_4HZ};

const expresslrs_mod_settings_s ExpressLRS_AirRateConfig[5] = {RF_RATE_200HZ, RF_RATE_100HZ, RF_RATE_50HZ, RF_RATE_25HZ, RF_RATE_4HZ};

expresslrs_mod_settings_s ExpressLRS_currAirRate;
expresslrs_mod_settings_s ExpressLRS_prevAirRate;

#define MaxPower100mW_Module 20
#define MaxPower1000mW_Module 30
#define RF_Gain 10

int8_t ExpressLRS_currPower = 0;
int8_t ExpressLRS_prevPower = 0;

#define PREVENT_BIND_WHEN_CONNECTED true
