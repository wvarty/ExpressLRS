#include "common.h"

void commonConfig()
{ //settings common to both master and slave
}

extern SX127xDriver Radio;

// TODO: Validate values for RFmodeCycleAddtionalTime and RFmodeCycleInterval for rates lower than 50HZ
expresslrs_mod_settings_s RF_RATE_200HZ =  {BW_500_00_KHZ, SF_6,  CR_4_5, -112, 5000,   200, TLM_RATIO_1_64,   4, 8,  RATE_200HZ, 1000, 1500};
expresslrs_mod_settings_s RF_RATE_100HZ =  {BW_500_00_KHZ, SF_7,  CR_4_7, -117, 10000,  100, TLM_RATIO_1_32,   4, 8, RATE_100HZ, 2000, 2000};
expresslrs_mod_settings_s RF_RATE_50HZ =   {BW_500_00_KHZ, SF_8,  CR_4_7, -120, 20000,  50,  TLM_RATIO_1_16,   2, 10, RATE_50HZ, 6000, 2500};
expresslrs_mod_settings_s RF_RATE_25HZ =   {BW_250_00_KHZ, SF_8,  CR_4_7, -123, 40000,  25,  TLM_RATIO_NO_TLM, 2, 8,  RATE_25HZ, 6000, 2500};
expresslrs_mod_settings_s RF_RATE_4HZ =    {BW_250_00_KHZ, SF_11, CR_4_5, -131, 250000, 4,   TLM_RATIO_NO_TLM, 2, 8,  RATE_4HZ, 6000, 2500};

const expresslrs_mod_settings_s ExpressLRS_AirRateConfig[5] = {RF_RATE_200HZ, RF_RATE_100HZ, RF_RATE_50HZ, RF_RATE_25HZ, RF_RATE_4HZ};

expresslrs_mod_settings_s ExpressLRS_nextAirRate;
expresslrs_mod_settings_s ExpressLRS_currAirRate;
expresslrs_mod_settings_s ExpressLRS_prevAirRate;

int8_t ExpressLRS_currPower = 0;
int8_t ExpressLRS_prevPower = 0;

connectionState_e connectionState = disconnected;
connectionState_e connectionStatePrev = disconnected;

//uint8_t UID[6] = {48, 174, 164, 200, 100, 50};
//uint8_t UID[6] = {180, 230, 45, 152, 126, 65}; //sandro mac
uint8_t UID[6] = {180, 230, 45, 152, 125, 173}; // Wez's MAC

uint8_t CRCCaesarCipher = UID[4];
uint8_t DeviceAddr = UID[5] & 0b111111;

uint8_t BindingCipher = 100;
uint8_t BindingAddr = 50 & 0b111111;

bool FreqLocked = false;

#define RSSI_FLOOR_NUM_READS 5 // number of times to sweep the noise foor to get avg. RSSI reading
#define MEDIAN_SIZE 20

int16_t MeasureNoiseFloor()
{
    int NUM_READS = RSSI_FLOOR_NUM_READS * NumOfFHSSfrequencies;
    float returnval = 0;

    for (int freq = 0; freq < NumOfFHSSfrequencies; freq++)
    {
        FHSSsetCurrIndex(freq);
        Radio.SetMode(SX127X_CAD);

        for (int i = 0; i < RSSI_FLOOR_NUM_READS; i++)
        {
            returnval = returnval + Radio.GetCurrRSSI();
            delay(5);
        }
    }
    returnval = returnval / NUM_READS;
    return (returnval);
}

uint8_t TLMratioEnumToValue(expresslrs_tlm_ratio_e enumval)
{
    switch (enumval)
    {
    case TLM_RATIO_NO_TLM:
        return 0;
        break;
    case TLM_RATIO_1_2:
        return 2;
        break;
    case TLM_RATIO_1_4:
        return 4;
        break;
    case TLM_RATIO_1_8:
        return 8;
        break;
    case TLM_RATIO_1_16:
        return 16;
        break;
    case TLM_RATIO_1_32:
        return 32;
        break;
    case TLM_RATIO_1_64:
        return 64;
        break;
    case TLM_RATIO_1_128:
        return 128;
        break;
    default:
        return 0xFF;
    }
}