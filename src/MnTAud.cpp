		/*
		Arduino MKR1000 SAMD21E18H Audio Recording Library (MnTAud)
		For testing purposes only.
		Ver. 0.5. First Commit. 2 August 2020.
		#####
		Simple Audio Recording Library for SAMD21 boards.
		Utilises CMSIS and ASF.
		*/

	/// KIV: Should set higher prescaler values.
	/// KIV: Do check for max freq allowed.

/***************************************************/

#if defined (ARDUINO_ARCH_SAMD)

#include "MnTAud.h"

FatFile myFile;

SdFat SD;


	/*
	Variables
	*/

volatile uint8_t adcBuffer[2][MAXBUFFSIZE];
volatile int adcBuffernum = 0;
volatile int count = 0;
volatile int killswitch = 0;
void* p;

#include <Arduino.h>
#define DEBUGLIBRARY 0

uint32_t MnTAud::sampleFreq = 8000;
bool MnTAud::inoperation;


	/*
	Public Functions
	*/

void MnTAud::SystemAudStart() {

	// Initialises all required peripherals and systems within the scope of this library.
	// Currently just test standard settings. 

	adcSetup(0x03, 0x02);
	// Serial.println("ADC setup!");
	adcInit();
	// Serial.println("ADC initialised!");
	// adcConfigInterrupt(true, 0x00);
	// Serial.println("ADC Config initialised!");
	timerSetup(0x03, 8000);

	// Serial.println("ADC and TC initialised.");
}

void MnTAud::createMyFile(const char *FileCreate) {

	inoperation = true;

	// Creates file.
	myFile.open(FileCreate, O_CREAT | O_WRITE);
	// Serial.println("File Create 0");

	if(!myFile.isOpen()){
		// Serial.println("Failed to open for writing!");
		return;
	}

	myFile.write((byte*)"RIFF    WAVEfmt ",16);
	// Serial.println("Written!");
	myFile.close();
	// Serial.println("Closed!");

	inoperation = false;
}

void MnTAud::fillWavHeader(const char *FileRec, uint16_t sampleFreq) {

	// File must not be open.
	if (myFile.isOpen()) {
		// Serial.println("Close the file before accessing other operations.");
		return;
	}

	// Open file.
	myFile.open(FileRec, O_WRITE); // Do not use FILE_WRITE since it includes O_APPEND which interferes with seekSet().

	// If file is empty, abort.
	if (myFile.fileSize() <= 44) {
		// Serial.println("The file is empty. Fill the file with PCM data.");
		myFile.close();
		return;
	}

	// Edit header details.
	// numChannels and bitsPerSample predefined as in overhead.
	WavHeader	CurrentHeader;
	CurrentHeader.chunkSize = myFile.fileSize() - 8;
	CurrentHeader.sampleRate = sampleFreq;
	CurrentHeader.byteRate = sampleFreq * CurrentHeader.numChannels * CurrentHeader.bitsPerSample / 8;
	CurrentHeader.blockAlign = CurrentHeader.numChannels * CurrentHeader.bitsPerSample / 8;
	CurrentHeader.subChunk2Size = myFile.fileSize() - 44;

	// Point to start of file address and write WavHeader.
	myFile.seekSet(0);
	myFile.write((uint8_t*)&CurrentHeader, 44);

	// Close file to sync from cache to memory.
	myFile.close();
}

void MnTAud::StartRecording(const char *FileRec, uint16_t sampleFreq) {

	inoperation = true;
	timerSetup(0x03, sampleFreq);

	// File must not be open.
	if (myFile.isOpen()) {
		// Serial.println("Close the file before accessing other operations.");
		inoperation = false;
		return;
	}

	// Open the file. Note that O_APPEND is active, setting cursor position to EOF with every write.
	myFile.open(FileRec, O_CREAT | O_WRITE);

	// File must be empty.
		if (myFile.fileSize() != 0) {
		// Serial.println("The file already exists!");
		return;
	}


	if (!myFile.isOpen()) {
		// Serial.println("Failed to initialise the file.");
		inoperation = false;
		return;
	}

	// Initialise "cursor position" at start of file address.
	myFile.seekSet(0);
	// Serial.println("Check!");

	// Create a placeholder 44 bits for the WAV header. Cursor should be at start of data chunk by the end.
	for (int i = 0; i < 44; i++) {
		myFile.write((uint8_t) 0x00);
	}
	// Serial.println("Check 2!");

	// Get and discard first dummy ADC call.
	adcRead();
	// Serial.println("Check 3!");

	// Enable timer counter to feed via DMA
	// Serial.println("Recording begin!");
	timerEnable();
}

void MnTAud::StopRecording(const char *FileRec, uint16_t sampleFreq) {

	// Serial.println("Disabling!");
	// Disable timer interrupt routine.
	timerDisable();
	// Serial.println("Check 4!");

	// Flush the data stream and close the file to write to memory.
	myFile.sync();
	// Serial.println("Check 5!");
	myFile.truncate(myFile.curPosition());
	// Serial.println("Check 6!");
	myFile.close();
	// Serial.println("Check 7!");

	// Create WAV file header.
	fillWavHeader(FileRec, sampleFreq);
	// Serial.println("Check 8!");
	inoperation = false;
	// Serial.println("Success!");
}


	/*
	Private Functions
	*/

//////////////////
//
// Analog-Digital Converter
//
//////////////////////

void MnTAud::adcSetup(uint32_t Clock, uint32_t DivFactor) {

	// Via the Power Manager module, enable the APBC mask for the ADC.
	    //// This should already be ON by default, but it is good to include.
	PM->APBCMASK.reg |= PM_APBCMASK_ADC;

	// Select a generic clock and configure its division factor.
	    //// For clock 3, it has 8 division factor bits, DIV[7:0]. Suppose DivFactor of 2.
	GCLK->GENDIV.reg |= GCLK_GENDIV_ID(Clock) | GCLK_GENDIV_DIV(DivFactor);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	// Configure the generic clock generator with an internal source.
	    //// Here we've chosen the 8MHz oscillator.
    GCLK->GENCTRL.reg |= GCLK_GENCTRL_ID(Clock) | GCLK_GENCTRL_SRC_OSC8M | GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

	// Enable the generic clock and configure it to the ADC.
	//// Clock ID 0x1E is GCLK_ADC.
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(0x1E) | GCLK_CLKCTRL_GEN(Clock) | GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

void MnTAud::adcInit() {

	// Reset the ADC to initial state and disable it.
	ADC->CTRLA.reg = ADC_CTRLA_SWRST;

	// Select reference voltage for ADC. Enable Reference Buffer Offset Compensation to increase accuracy of gain if necessary.
	    //// 0.5VCC internal reference is used here.
	ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;	// | ADC_REFCTRL_REFCOMP

	// Set averaging control to 3 samples to reduce noise ratio.
	// ADC->AVGCTRL.reg |= ADC_AVGCTRL_SAMPLENUM_1;
	// ADC->AVGCTRL.bit.SAMPLENUM = 0x3;
	// ADC->AVGCTRL.bit.ADJRES = 0x3;

	// Configure input gain, negative and positive Mux.
	    //// Gain at 1X, internal ground as negative while ADC AIN0 Pin as positive.
	ADC->INPUTCTRL.reg |= ADC_INPUTCTRL_GAIN_DIV2 | ADC_INPUTCTRL_MUXNEG_GND | ADC_INPUTCTRL_MUXPOS_PIN0;
	while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);

	ADC->CTRLB.bit.FREERUN = 0;
	while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);

	// Configure prescaler and conversion settings for ADC.
	    //// ADC Conversion gives 8-bit resolution in single-ended mode. Peripheral clock divided by 4, giving 500kHz. 2.1MHz is max operating limit.
	ADC->CTRLB.reg = ADC_CTRLB_RESSEL_8BIT | ADC_CTRLB_PRESCALER_DIV4;
	while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);
        // 8-bit ADC mesasurement takes 5 clock cycles in single-ended mode. Sample rate is maximum 500kHz / 5 == 100 kHz.

    // Calibrate for gain and linearity offset using factory values. (NOTE: THERE ARE NO FACTORY VALUES IN ARD_MKR1000!)
	//// uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
	//// uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
	//// linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;
	//// while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);
	//// ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

	// Enable the ADC.
	ADC->CTRLA.reg = ADC_CTRLA_ENABLE;
}

/* void MnTAud::adcConfigInterrupt(bool enabled, uint32_t priority) {

	// Disable for configuration.
	NVIC_DisableIRQ(ADC_IRQn);
	if (enabled) {

		// Flush and enable ADC interrupt service in Nested Vector Interrupt Controller and set priority.
		NVIC_ClearPendingIRQ(ADC_IRQn);
		NVIC_EnableIRQ(ADC_IRQn);
		NVIC_SetPriority(ADC_IRQn, priority);

		// Configure for ADC READY interrupt.
		ADC->INTENSET.reg |= ADC_INTENSET_RESRDY;
		while (ADC->STATUS.reg & ADC_STATUS_SYNCBUSY);
	}
} */

uint32_t MnTAud::adcRead() {

	// Trigger ADC Conversion.
	ADC->SWTRIG.bit.START = 1;

	// Wait for Result Ready.
	while (ADC->INTFLAG.bit.RESRDY == 0);

	// Clear the flag and read result.
	ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

	return ADC->RESULT.reg;
}

//////////////////
//
// Timer Counter
//
//////////////////////

void MnTAud::timerSetup(uint32_t Clock, uint16_t sFreq) {

	// Configures Timer Counter 5 to call ADC.

	// Select a generic clock generator as source.
	///// Clock mask ID 0x1C is for TC4 and TC5.
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(0x1C) | GCLK_CLKCTRL_GEN(Clock) | GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.reg & ADC_STATUS_SYNCBUSY);

	// Enable the APBC clock for TC5.
	PM->APBCMASK.reg |= PM_APBCMASK_TC5;

	// Reset TC5.
	TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (TC5->COUNT16.CTRLA.reg & TC_STATUS_SYNCBUSY);
	while (TC5->COUNT16.CTRLA.bit.SWRST);
	// Serial.println("Clear 2");

	// Configure TC5 to 16 bit.
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
	// Serial.println("Clear 3");

	// Set TC5 waveform generation to Match Frequency.
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	// Serial.println("Clear 4");

	// Configure prescaler for TC5.
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1; /// Make variable?
	// Serial.println("Clear 5");

	// Configure the compare-capture register for fitting to requested sampling frequency.
	//// Set as 16-bit as this is a 16-bit counter.
	////// Affected by adcSetup's configuring of GCLK generator if same used.
	////// As in above example, clock is 8MHz. DivFactor of GENCLK is 2, bringing it to 4MHz. Prescaler is set to 1 as above.
	TC5->COUNT16.CC[0].reg = (uint16_t) (4000000 / (1 * (sFreq - 1)));
	while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
	// Serial.println("Clear 6");

	// Configure and enable TC5 interrupt NVIC line.
	//// Priority set at 0.
	NVIC_DisableIRQ(TC5_IRQn);
	NVIC_ClearPendingIRQ(TC5_IRQn);
	NVIC_SetPriority(TC5_IRQn, 0);
	NVIC_EnableIRQ(TC5_IRQn);
	
	// Enable TC5 compare mode interrupt genertion.
	TC5->COUNT16.INTENSET.bit.MC0 = 1; // 1 to enable
	while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
	// Serial.println("Clear 7");
}

void MnTAud::timerEnable(){

	// Enable TC5.
	TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
}

void TC5_Handler() {

	// Read ADC value into buffer.
	adcBuffer[adcBuffernum][count] = MnTAud::adcRead();

	// Wait for ADC to finish operation if necessary.
	while (ADC->STATUS.bit.SYNCBUSY == 1);
	count++;

	// Execute double-buffering when buffer is full.
	if (count >= MAXBUFFSIZE) {
		// Store address of filled buffer.
		/*for(uint32_t i=0; i<MAXBUFFSIZE; i++) {
    		// Serial.print(adcBuffer[adcBuffernum][i], HEX); // Serial.print(' ');
    		if ((i & 15) == 15) // Serial.println(); }
		// Serial.println("Current Buffer is: ");
		// Serial.println(adcBuffernum);
		// Serial.println((unsigned long) &adcBuffer[adcBuffernum]);*/
		p = (void *) &adcBuffer[adcBuffernum];
		adcBuffernum = (adcBuffernum + 1) % 2;
		/*// Serial.println("New Buffer is: ");
		// Serial.println(adcBuffernum);
		// Serial.println((unsigned long) &adcBuffer[adcBuffernum]);
		// Serial.println("Selected Buffer Address is: ");
		// Serial.println((unsigned long) p);*/
		count = 0;
		myFile.write((byte*) p, 512); // Trigger DMA to SPI write. ////////////// STALLING POINT (check driver)
		//// Serial.println("Sent Buffer!");
		// killswitch++;
		// Retrigger TC5. /// Time lapse???
		//TC5->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
		//while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);
	}
	
	// Retrigger the flag.
	TC5->COUNT16.INTFLAG.bit.MC0 = 1; 
}

void MnTAud::timerDisable() {
	
	// Disable interrupts.
	NVIC_DisableIRQ(TC5_IRQn);
	// Serial.println("Check 4A!");
	NVIC_ClearPendingIRQ(TC5_IRQn);
	// Serial.println("Check 4B!");

	// Disable TC5.
	TC5->COUNT16.CTRLA.bit.ENABLE = 0x00;
	// Serial.println("Check 4C!");

	// Reset TC5.
	TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (TC5->COUNT16.CTRLA.reg & TC_STATUS_SYNCBUSY);
	// Serial.println("Check 4D!");
	while (TC5->COUNT16.CTRLA.bit.SWRST);
	// Serial.println("Check 4E!");
}

#endif
