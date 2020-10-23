		/*
		Arduino MKR1000 SAMD21E18H Audio Recording Library (MnTAud)
        For testing purposes only.
        Ver. 0.5. First Commit. 2 August 2020.
		#####
		Simple Audio Recording Library for SAMD21 boards.
		Utilises CMSIS and ASF.
		*/

/***************************************************/

#ifndef MnTAud_h
#define MnTAud_h

// #include <SPI.h> // if not already executed in main sketch.
#include <SdFat.h>
#include <Adafruit_ZeroDMA.h>
#include <Arduino.h>

#define MAXBUFFSIZE 512

class MnTAud
{
    public:

        static uint32_t sampleFreq;
        static bool inoperation;
        static uint32_t adcRead();

        void SystemAudStart();
        void createMyFile(const char* FileCreate);
        void fillWavHeader(const char* FileRec, uint16_t sampleFreq);
        void StartRecording(const char* FileRec, uint16_t sampleFreq);
        void StopRecording(const char* FileRec, uint16_t sampleFreq);

    private:

        struct WavHeader {
            const char chunkID[4] = {'R','I','F','F'};
            uint32_t chunkSize;                     // Size of (entire file in bytes - 8 bytes) or (data size + 36)
            const char format[4] = {'W','A','V','E'};
            const char subchunkID[4] = {'f','m','t',' '};
            const uint32_t subchunkSize = 16;
            const uint16_t audioFormat = 1;              // PCM == 1
            uint16_t numChannels = 1;                    // 1 = Mono, 2 = Stereo
            uint32_t sampleRate;        
            uint32_t byteRate;                   // == SampleRate * NumChannels * BitsPerSample/8
            uint16_t blockAlign = 1;                     // == NumChannels * BitsPerSample/8
            uint16_t bitsPerSample = 8;                  // 8 or 16.
            const char subChunk2ID[4]= {'d','a','t','a'};
            uint32_t subChunk2Size = 0;                  // == NumSamples * NumChannels * BitsPerSample/8
            //Data                                       // Audio data.
        };

        void adcSetup(uint32_t Clock, uint32_t DivFactor);
        void adcInit();
        // void adcConfigInterrupt(bool enabled, uint32_t priority);

        void timerSetup(uint32_t Clock, uint16_t sFreq);
        void timerEnable();
        void timerDisable();
;

};

#endif
