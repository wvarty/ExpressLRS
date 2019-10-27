#include "common.h"

extern SX127xDriver Radio;

uint8_t TxBaseMac[6] = {48, 174, 164, 200, 100, 50};

uint8_t DeviceAddr = TxBaseMac[5] & 0b111111; // temporarily based on mac until listen before assigning method merged

expresslrs_mod_settings_s RF_RATE_200HZ = {BW_500_00_KHZ, SF_6, CR_4_5, 5000, 200, 64, 8, 8, RATE_200HZ};
expresslrs_mod_settings_s RF_RATE_100HZ = {BW_500_00_KHZ, SF_7, CR_4_7, 10000, 100, 32, 4, 10, RATE_100HZ};
expresslrs_mod_settings_s RF_RATE_50HZ = {BW_500_00_KHZ, SF_8, CR_4_7, 20000, 50, 16, 2, 10, RATE_50HZ};
expresslrs_mod_settings_s RF_RATE_25HZ = {BW_250_00_KHZ, SF_8, CR_4_7, 40000, 25, 0, 2, 8, RATE_25HZ};
expresslrs_mod_settings_s RF_RATE_4HZ = {BW_250_00_KHZ, SF_11, CR_4_5, 250000, 4, 0, 2, 8, RATE_4HZ};

expresslrs_mod_settings_s ExpressLRS_currAirRate;
expresslrs_mod_settings_s ExpressLRS_prevAirRate;

#define RSSI_FLOOR_NUM_READS 5 // number of times to sweep the noise foor to get avg. RSSI reading
#define MEDIAN_SIZE 50

int16_t MeasureRSSI(int FHSSindex)
{
    FHSSsetCurrIndex(FHSSindex);
    Radio.SetFrequency(FHSSgetCurrFreq());
    delay(1);
    return Radio.GetLastPacketRSSI();
}

int16_t MeasureNoiseFloor()
{ // read multiple values and sort them to take the mode ///adapted from :http://www.elcojacobs.com/eleminating-noise-from-sensor-readings-on-arduino-with-digital-filtering/

    int sortedValues[RSSI_FLOOR_NUM_READS * NumOfFHSSfrequencies];

    for (int freq = 0; freq < NumOfFHSSfrequencies; freq++)
    {
        for (int i = 0; i < RSSI_FLOOR_NUM_READS; i++)
        {
            ///generate an array NUM_READS in size and fill it with readings sorted from largest-smallest
            int value = MeasureRSSI(freq);
            int j;
            if (value < sortedValues[0] || i == 0)
            {
                j = 0; //insert at first position
            }
            else
            {
                for (j = 1; j < i; j++)
                {
                    if (sortedValues[j - 1] <= value && sortedValues[j] >= value)
                    {
                        // j is insert position
                        break;
                    }
                }
            }
            for (int k = i; k > j; k--)
            {
                // move all values higher than current reading up one position
                sortedValues[k] = sortedValues[k - 1];
            }
            sortedValues[j] = value; //insert current reading
        }
    }

    //return scaled mode of MEDIAN_SIZE values
    float returnval = 0;
    for (int i = RSSI_FLOOR_NUM_READS / 2 - (MEDIAN_SIZE / 2); i < (RSSI_FLOOR_NUM_READS / 2 + (MEDIAN_SIZE / 2)); i++)
    {
        returnval += sortedValues[i];
    }
    returnval = returnval / MEDIAN_SIZE;
    //return returnval*1100/1023;
    return int(returnval);
}