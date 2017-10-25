/*

This sketch illustrates how to set a timer on an SAMD21 based board in Arduino (Feather M0, Arduino Zero should work)

*/

const int dacPIN = DAC0;  //Assign Output Pin to Pin A0

uint32_t sRate = 1; //This determines how many times per second TC5_Handler() gets called per second (It is the sample rate of the sine wave in Hertz)



float Ts; //Declaring Period Variable



float f; //Declaring frequency variable





void setup() {

  uint16_t Fs = 100000;    //Declaring the Sample Rate Value

  analogWriteResolution(10);  //Setting DAC Resolution to its maximum value for the Arduino Zero in this case this is 10 bits

  Ts = 1.0/(float)Fs;         //Calculating Period from the Sample rate value

  f = 1000;       //Declaring the clock frequency value

  tcConfigure(f); //configuring the timer to run at (f)Hertz

  tcStartCounter(); //Initalizes the timer

}



void loop() {

  



    

}



//this function gets called by the interrupt at <sampleRate>Hertz

void TC5_Handler (void) {

  //YOUR CODE HERE 

  

   static uint16_t i = 0;    // Declaring variable n as an unsigned 16-bit integer

   uint16_t sig = (uint16_t)(511.5*(cos(2*PI*f*i*Ts)+1.0));   //Declaring signal variable sig, assigning a value of cos with the parameters provided in the instructions all added with 1 to give a positive value and multiplied by 511.5 (1023/2) to properly scale the output for the DAC

   analogWrite(dacPIN, sig);  //Writing the output through the DAC to dacPIN (pin A0)to be the values determine by function sig

   i++;                     //Incrementing n so that way we have diffrent vaules of sig (cosine F(x)) to work with

   if (i>65535) i = 0;      //65536 is 2^16, maximum value of a 16-bit number. Once i reaches 65535 the value of i is reseted to 0

   

  // END OF YOUR CODE

  

  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //TIMER CODE. DON'T TOUCH!

}



/* 

 *  TIMER SPECIFIC FUNCTIONS FOLLOW

 *  you shouldn't change these unless you know what you're doing

 */



//Configures the TC to generate output events at the sample frequency.

//Configures the TC in Frequency Generation mode, with an event output once

//each time the audio sample frequency period expires.

 void tcConfigure(int sampleRate)

{

 // Enable GCLK for TCC2 and TC5 (timer counter input clock)

 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;

 while (GCLK->STATUS.bit.SYNCBUSY);



 tcReset(); //reset TC5



 // Set Timer counter Mode to 16 bits

 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

 // Set TC5 mode as match frequency

 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

 //set prescaler and enable TC5

 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;

 //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform

 TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate / 1024 - 1);

 while (tcIsSyncing());

 

 // Configure interrupt request

 NVIC_DisableIRQ(TC5_IRQn);

 NVIC_ClearPendingIRQ(TC5_IRQn);

 NVIC_SetPriority(TC5_IRQn, 0);

 NVIC_EnableIRQ(TC5_IRQn);



 // Enable the TC5 interrupt request

 TC5->COUNT16.INTENSET.bit.MC0 = 1;

 while (tcIsSyncing()); //wait until TC5 is done syncing 

} 



//Function that is used to check if TC5 is done syncing

//returns true when it is done syncing

bool tcIsSyncing()

{

  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;

}



//This function enables TC5 and waits for it to be ready

void tcStartCounter()

{

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register

  while (tcIsSyncing()); //wait until snyc'd

}



//Reset TC5 

void tcReset()

{

  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;

  while (tcIsSyncing());

  while (TC5->COUNT16.CTRLA.bit.SWRST);

}



//disable TC5

void tcDisable()

{

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;

  while (tcIsSyncing());

}
