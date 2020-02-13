#include "FHSS.h"
#include "debug.h"

uint8_t volatile FHSSptr = 0;
uint8_t FHSSsequence[256] = {0};

#ifdef Regulatory_Domain_AU_915

uint8_t NumOfFHSSfrequencies = 20;

int32_t FreqCorrection = 0;

#elif defined Regulatory_Domain_AU_433

uint8_t NumOfFrequencies = 3;

#endif

void ICACHE_RAM_ATTR FHSSsetCurrIndex(uint8_t value)
{ // get the current index of the FHSS pointer
    FHSSptr = value;
}

uint8_t ICACHE_RAM_ATTR FHSSgetCurrIndex()
{ // get the current index of the FHSS pointer
    return FHSSptr;
}

uint32_t ICACHE_RAM_ATTR GetInitialFreq()
{
#ifdef Regulatory_Domain_AU_915

    return FHSSfreqs915[0] - FreqCorrection;

#elif defined Regulatory_Domain_AU_433

    return FHSSfreqs433[0] - FreqCorrection;

#endif
}

uint32_t ICACHE_RAM_ATTR FHSSgetCurrFreq()
{
#ifdef Regulatory_Domain_AU_915

    return FHSSfreqs915[FHSSsequence[FHSSptr]] - FreqCorrection;

#elif defined Regulatory_Domain_AU_433

    return FHSSfreqs433[FHSSsequence[FHSSptr]] - FreqCorrection;

#endif

    return 0;
}

uint32_t ICACHE_RAM_ATTR FHSSgetNextFreq()
{
    FHSSptr++;

    return FHSSgetCurrFreq();
}

void ICACHE_RAM_ATTR FHSSrandomiseFHSSsequence()
{

    DEBUG_PRINT("FHSSsequence[] = ");

    // TODO: Check that removing UID[2] still gives a good seed
    long uidSeed = ((long)UID[2] << 24) + ((long)UID[3] << 16) + ((long)UID[4] << 8) + UID[5];
    rngSeed(uidSeed);

#ifdef Regulatory_Domain_AU_915

    int hopSeqLength = 256;
    int numOfFreqs = 19;
    int limit = floor(hopSeqLength / numOfFreqs);

    int prev_val = 0;
    int rand = 0;

    int last_InitialFreq = 0;
    int last_InitialFreq_interval = numOfFreqs;

    int tracker[20] = {0};

    for (int i = 0; i < hopSeqLength; i++)
    {

        if (i >= (last_InitialFreq + last_InitialFreq_interval))
        {
            rand = 0;
            last_InitialFreq = i;
        }
        else
        {
            while (prev_val == rand || tracker[rand] >= limit || rand == 0 || rand > numOfFreqs)
            {
                rand = rng5Bit();
            }
        }

        FHSSsequence[i] = rand;
        tracker[rand]++;
        prev_val = rand;

        DEBUG_PRINT(FHSSsequence[i]);
        DEBUG_PRINT(", ");
    }

#elif defined Regulatory_Domain_AU_433

    int prev_val = rng0to2(); // Randomised so that FHSSsequence[0] can also be 0.
    int rand = 0;

    for (int i = 0; i < 256; i++)
    {
        while (prev_val == rand)
        {
            rand = rng0to2();
        }

        prev_val = rand;
        FHSSsequence[i] = rand;

        DEBUG_PRINT(FHSSsequence[i]);
        DEBUG_PRINT(", ");
    }

#endif

    DEBUG_PRINTLN("");
}