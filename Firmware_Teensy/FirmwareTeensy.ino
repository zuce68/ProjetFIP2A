#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

#define SWITCH1       0 //active filter1
#define SWITCH2       1 //active filter2
#define SWITCH3       2 //soft white noise
#define SWITCH4       3 //hardcore white noise
#define SWITCH5       4 //polarity inversion
#define NUM_COEFFS1 200 //approximativly 10% of cpu per FIR filter
#define NUM_COEFFS2 200 //Filter coefficients, must be an even number !!!!!



uint8_t   switch1_state                 = 0;
uint8_t   switch2_state                 = 0;
uint8_t   switch3_state                 = 0;
uint8_t   switch4_state                 = 0;
uint8_t   switch5_state                 = 0;
uint8_t   prev_switch1_state            = 0;
uint8_t   prev_switch2_state            = 0;
uint8_t   prev_switch3_state            = 0;
uint8_t   prev_switch4_state            = 0;
uint8_t   prev_switch5_state            = 0;
uint8_t   prev_state_signal_detected    = 0;
uint64_t  noise_delay                   = 0;
bool      signal_detected               = false;
short int FIR1_Coeff[NUM_COEFFS1];
short int FIR2_Coeff[NUM_COEFFS2];



AudioEffectMultiply      multiply1;      
AudioInputI2S            i2s1;           
AudioFilterFIR           fir1;           
AudioMixer4              mixer1;         
AudioAnalyzePeak         peak1;          
AudioFilterFIR           fir2;           
AudioSynthNoiseWhite     noise3; 
AudioSynthNoiseWhite     noise2; 
AudioSynthNoiseWhite     noise5; 
AudioMixer4              mixer2;         
AudioMixer4              mixer3;         
AudioSynthNoiseWhite     noise4; 
AudioMixer4              mixer4; 
AudioSynthWaveformDc     dc1;            
AudioEffectMultiply      multiply2;      
AudioMixer4              mixer5; 
AudioOutputI2S           i2s2;           
AudioConnection          patchCord1(i2s1, 0, fir1, 0);
AudioConnection          patchCord2(i2s1, 0, mixer1, 0);
AudioConnection          patchCord3(i2s1, 0, peak1, 0);
AudioConnection          patchCord4(fir1, 0, mixer1, 1);
AudioConnection          patchCord5(mixer1, 0, mixer2, 0);
AudioConnection          patchCord6(mixer1, fir2);
AudioConnection          patchCord7(fir2, 0, mixer2, 1);
AudioConnection          patchCord8(noise3, 0, mixer3, 1);
AudioConnection          patchCord9(noise2, 0, mixer3, 2);
AudioConnection          patchCord10(noise5, 0, mixer3, 3);
AudioConnection          patchCord11(mixer2, 0, mixer3, 0);
AudioConnection          patchCord12(mixer3, 0, mixer4, 0);
AudioConnection          patchCord13(noise4, 0, mixer4, 1);
AudioConnection          patchCord14(mixer4, 0, multiply2, 0);
AudioConnection          patchCord15(mixer4, 0, mixer5, 0);
AudioConnection          patchCord16(dc1, 0, multiply2, 1);
AudioConnection          patchCord17(multiply2, 0, mixer5, 1);
AudioConnection          patchCord18(mixer5, 0, i2s2, 0);
AudioConnection          patchCord19(mixer5, 0, i2s2, 1);



AudioAmplifier amp;
AudioControlSGTL5000     ctl;            


const double coeff_fir1_double[] =
{
 -2.29948232e-03,  1.37082480e-03,  1.25014374e-03,  1.20644984e-03,
  1.13339559e-03,  9.61756913e-04,  6.63851423e-04,  2.59895712e-04,
 -1.91706183e-04, -6.04748867e-04, -8.89773654e-04, -9.73511424e-04,
 -8.23383241e-04, -4.55266940e-04,  5.95096505e-05,  6.12428308e-04,
  1.07267927e-03,  1.32459550e-03,  1.28663442e-03,  9.45554989e-04,
  3.51880328e-04, -3.76556557e-04, -1.07151255e-03, -1.58256393e-03,
 -1.75979789e-03, -1.53769761e-03, -9.37236240e-04, -7.16379212e-05,
  8.80251017e-04,  1.70124584e-03,  2.18983532e-03,  2.20420723e-03,
  1.70516182e-03,  7.69224859e-04, -4.16681513e-04, -1.59550126e-03,
 -2.48994634e-03, -2.87146756e-03, -2.61133255e-03, -1.72473742e-03,
 -3.70081455e-04,  1.16877930e-03,  2.54852850e-03,  3.43629083e-03,
  3.58705894e-03,  2.91574600e-03,  1.51712040e-03, -3.36336733e-04,
 -2.24578128e-03, -3.77006966e-03, -4.52794542e-03, -4.28800787e-03,
 -3.03377512e-03, -9.82711384e-04,  1.44676676e-03,  3.71787687e-03,
  5.29087878e-03,  5.74945879e-03,  4.90395409e-03,  2.85422868e-03,
 -1.52475254e-05, -3.09905931e-03, -5.69899093e-03, -7.17250128e-03,
 -7.09025017e-03, -5.34862839e-03, -2.21790720e-03,  1.68664768e-03,
  5.52201829e-03,  8.39189615e-03,  9.54689529e-03,  8.56840430e-03,
  5.49425015e-03,  8.48559632e-04, -4.43547540e-03, -9.19004705e-03,
 -1.22572710e-02, -1.27516464e-02, -1.02874625e-02, -5.11675190e-03,
  1.86256136e-03,  9.23725815e-03,  1.53389942e-02,  1.85810831e-02,
  1.78207380e-02,  1.26585669e-02,  3.62389789e-03, -7.81469233e-03,
 -1.94244572e-02, -2.85487870e-02, -3.25632376e-02, -2.93681483e-02,
 -1.78235574e-02,  1.95525957e-03,  2.85037313e-02,  5.91451127e-02,
  9.03471707e-02,  1.18236856e-01,  1.39197496e-01,  1.50437089e-01,
  1.50437089e-01,  1.39197496e-01,  1.18236856e-01,  9.03471707e-02,
  5.91451127e-02,  2.85037313e-02,  1.95525957e-03, -1.78235574e-02,
 -2.93681483e-02, -3.25632376e-02, -2.85487870e-02, -1.94244572e-02,
 -7.81469233e-03,  3.62389789e-03,  1.26585669e-02,  1.78207380e-02,
  1.85810831e-02,  1.53389942e-02,  9.23725815e-03,  1.86256136e-03,
 -5.11675190e-03, -1.02874625e-02, -1.27516464e-02, -1.22572710e-02,
 -9.19004705e-03, -4.43547540e-03,  8.48559632e-04,  5.49425015e-03,
  8.56840430e-03,  9.54689529e-03,  8.39189615e-03,  5.52201829e-03,
  1.68664768e-03, -2.21790720e-03, -5.34862839e-03, -7.09025017e-03,
 -7.17250128e-03, -5.69899093e-03, -3.09905931e-03, -1.52475254e-05,
  2.85422868e-03,  4.90395409e-03,  5.74945879e-03,  5.29087878e-03,
  3.71787687e-03,  1.44676676e-03, -9.82711384e-04, -3.03377512e-03,
 -4.28800787e-03, -4.52794542e-03, -3.77006966e-03, -2.24578128e-03,
 -3.36336733e-04,  1.51712040e-03,  2.91574600e-03,  3.58705894e-03,
  3.43629083e-03,  2.54852850e-03,  1.16877930e-03, -3.70081455e-04,
 -1.72473742e-03, -2.61133255e-03, -2.87146756e-03, -2.48994634e-03,
 -1.59550126e-03, -4.16681513e-04,  7.69224859e-04,  1.70516182e-03,
  2.20420723e-03,  2.18983532e-03,  1.70124584e-03,  8.80251017e-04,
 -7.16379212e-05, -9.37236240e-04, -1.53769761e-03, -1.75979789e-03,
 -1.58256393e-03, -1.07151255e-03, -3.76556557e-04,  3.51880328e-04,
  9.45554989e-04,  1.28663442e-03,  1.32459550e-03,  1.07267927e-03,
  6.12428308e-04,  5.95096505e-05, -4.55266940e-04, -8.23383241e-04,
 -9.73511424e-04, -8.89773654e-04, -6.04748867e-04, -1.91706183e-04,
  2.59895712e-04,  6.63851423e-04,  9.61756913e-04,  1.13339559e-03,
  1.20644984e-03,  1.25014374e-03,  1.37082480e-03, -2.29948232e-03
};

const double coeff_fir2_double[] =
{
  7.04924316e-04,  8.39840998e-05,  6.18553199e-05,  2.08462084e-05,
 -3.79318320e-05, -1.11054491e-04, -1.93904689e-04, -2.79796264e-04,
 -3.61555365e-04, -4.30704160e-04, -4.79381580e-04, -4.99379910e-04,
 -4.84527420e-04, -4.29823678e-04, -3.35347132e-04, -2.03902170e-04,
 -3.77677277e-05,  1.50813164e-04,  3.51458299e-04,  5.48968781e-04,
  7.27562183e-04,  8.70550024e-04,  9.62381595e-04,  9.89631365e-04,
  9.42610651e-04,  8.16641009e-04,  6.12687062e-04,  3.38028711e-04,
  5.95787507e-06, -3.63324135e-04, -7.45085159e-04, -1.11171622e-03,
 -1.43244347e-03, -1.67832427e-03, -1.82238948e-03, -1.84346090e-03,
 -1.72745864e-03, -1.46974361e-03, -1.07593582e-03, -5.62728243e-04,
  4.23789093e-05,  7.02253878e-04,  1.37218965e-03,  2.00275985e-03,
  2.54244924e-03,  2.94231586e-03,  3.15964935e-03,  3.16118030e-03,
  2.92748006e-03,  2.45457704e-03,  1.75644394e-03,  8.64918192e-04,
 -1.70693494e-04, -1.28620216e-03, -2.40580479e-03, -3.44674540e-03,
 -4.32484137e-03, -4.96071591e-03, -5.28606349e-03, -5.24918909e-03,
 -4.82080310e-03, -3.99768335e-03, -2.80505833e-03, -1.29759397e-03,
  4.42591997e-04,  2.30899889e-03,  4.17666747e-03,  5.90911657e-03,
  7.36739068e-03,  8.41943272e-03,  8.94996067e-03,  8.86994535e-03,
  8.12520897e-03,  6.70322607e-03,  4.63749718e-03,  2.00988523e-03,
 -1.05076515e-03, -4.37259123e-03, -7.74771195e-03, -1.09423218e-02,
 -1.37085329e-02, -1.57983527e-02, -1.69779524e-02, -1.70426045e-02,
 -1.58302154e-02, -1.32333174e-02, -9.20847427e-03, -3.78254669e-03,
  2.94484729e-03,  1.08029571e-02,  1.95551715e-02,  2.89087546e-02,
  3.85274246e-02,  4.80469827e-02,  5.70929115e-02,  6.52988028e-02,
  7.23249737e-02,  7.78755543e-02,  8.17141045e-02,  8.36757202e-02,
  8.36757202e-02,  8.17141045e-02,  7.78755543e-02,  7.23249737e-02,
  6.52988028e-02,  5.70929115e-02,  4.80469827e-02,  3.85274246e-02,
  2.89087546e-02,  1.95551715e-02,  1.08029571e-02,  2.94484729e-03,
 -3.78254669e-03, -9.20847427e-03, -1.32333174e-02, -1.58302154e-02,
 -1.70426045e-02, -1.69779524e-02, -1.57983527e-02, -1.37085329e-02,
 -1.09423218e-02, -7.74771195e-03, -4.37259123e-03, -1.05076515e-03,
  2.00988523e-03,  4.63749718e-03,  6.70322607e-03,  8.12520897e-03,
  8.86994535e-03,  8.94996067e-03,  8.41943272e-03,  7.36739068e-03,
  5.90911657e-03,  4.17666747e-03,  2.30899889e-03,  4.42591997e-04,
 -1.29759397e-03, -2.80505833e-03, -3.99768335e-03, -4.82080310e-03,
 -5.24918909e-03, -5.28606349e-03, -4.96071591e-03, -4.32484137e-03,
 -3.44674540e-03, -2.40580479e-03, -1.28620216e-03, -1.70693494e-04,
  8.64918192e-04,  1.75644394e-03,  2.45457704e-03,  2.92748006e-03,
  3.16118030e-03,  3.15964935e-03,  2.94231586e-03,  2.54244924e-03,
  2.00275985e-03,  1.37218965e-03,  7.02253878e-04,  4.23789093e-05,
 -5.62728243e-04, -1.07593582e-03, -1.46974361e-03, -1.72745864e-03,
 -1.84346090e-03, -1.82238948e-03, -1.67832427e-03, -1.43244347e-03,
 -1.11171622e-03, -7.45085159e-04, -3.63324135e-04,  5.95787507e-06,
  3.38028711e-04,  6.12687062e-04,  8.16641009e-04,  9.42610651e-04,
  9.89631365e-04,  9.62381595e-04,  8.70550024e-04,  7.27562183e-04,
  5.48968781e-04,  3.51458299e-04,  1.50813164e-04, -3.77677277e-05,
 -2.03902170e-04, -3.35347132e-04, -4.29823678e-04, -4.84527420e-04,
 -4.99379910e-04, -4.79381580e-04, -4.30704160e-04, -3.61555365e-04,
 -2.79796264e-04, -1.93904689e-04, -1.11054491e-04, -3.79318320e-05,
  2.08462084e-05,  6.18553199e-05,  8.39840998e-05,  7.04924316e-04
};


//transform the fir double coefficient to short int for teensy
void transformCoeffs( short int out[], const double in[], int k ){
  for ( int i = 0 ; i < k ; i++ )
    out[i] = 32768 * in[i];
}

//mixer inbitialized to passthrough mode (signal pass without any modification)
void signalPassthrough(){
  mixer1.gain(0,1);
  mixer1.gain(1,0);
  mixer2.gain(0,1);
  mixer2.gain(1,0);
  mixer3.gain(0,1);
  mixer3.gain(1,0);
  mixer3.gain(2,0);
  mixer3.gain(3,0);
  mixer4.gain(0,1);
  mixer4.gain(1,0);
  mixer5.gain(0,0);
  mixer5.gain(1,1);
}
void setup() {

  /*configuration of the teensy codec*/
  AudioMemory(160); 
  ctl.enable();
  ctl.volume(0.8);
  amp.gain(1);
  ctl.inputSelect(AUDIO_INPUT_LINEIN);

  /*configuration of audio objects*/
  //noise combine to the signal
  noise2.amplitude(0.2);
  noise3.amplitude(0.2);
  noise4.amplitude(0.2);
  noise5.amplitude(0.2);

  //dc1 is multiply with signal to interverted the polarity
  dc1.amplitude(-1);

  //transform the fir double coefficient to short int for teensy
  transformCoeffs( FIR1_Coeff, coeff_fir1_double, NUM_COEFFS1);
  transformCoeffs( FIR2_Coeff, coeff_fir2_double, NUM_COEFFS2);
  //start the fir filters
  fir1.begin(FIR1_Coeff, NUM_COEFFS1);
  fir2.begin(FIR2_Coeff, NUM_COEFFS2);


  /*Switch configuration*/
  pinMode(SWITCH1,INPUT_PULLUP);
  pinMode(SWITCH2,INPUT_PULLUP);
  pinMode(SWITCH3,INPUT_PULLUP);
  pinMode(SWITCH4,INPUT_PULLUP);
  pinMode(SWITCH5,INPUT_PULLUP);

  /*Programme initilalized in Passthrough mode*/
  signalPassthrough();

}
void loop() {


  /*Read the state of each switch*/
  switch1_state = digitalRead(SWITCH1);
  switch2_state = digitalRead(SWITCH2);
  switch3_state = digitalRead(SWITCH3);
  switch4_state = digitalRead(SWITCH4);
  switch5_state = digitalRead(SWITCH5);


  /*check if a signal has been detected*/
  if (peak1.available()) {
      int threshold = peak1.read() * 30.0;
      if(threshold>10){
        //Signal detected (here synchronization bit), we can now add noise, filter, etc. on the signal.
        signal_detected = true;
        noise_delay = 0;
      }
      else{
        noise_delay++;
        if(noise_delay>500){     
          //No signal, stop noise, filter, etc.                                
          signal_detected = false;
          noise_delay = 0;
        }
      }
        
  }
  if(signal_detected){
    //signal has been detected, we can apply noise, filter, etc.
    if((switch1_state!=prev_switch1_state) || (switch2_state!=prev_switch2_state) || (switch3_state!=prev_switch3_state) || (switch4_state!=prev_switch4_state) || (switch5_state!=prev_switch5_state) ||(signal_detected!=prev_state_signal_detected) )
    {
      //previous state is different from current state
      if(switch1_state){ 
        /*active filter 1 (soft) */
        mixer1.gain(0,0);
        mixer1.gain(1,1);
      }
      else{
        mixer1.gain(0,1);
        mixer1.gain(1,0);
      }
      
      if(switch2_state){ 
        /*active filter 2 (hard)*/
        mixer2.gain(0,0);
        mixer2.gain(1,1);
      }else{
        mixer2.gain(0,1);
        mixer2.gain(1,0);
      }

      if(switch3_state){ 
        /*active soft white noise*/
        mixer4.gain(0,0.3);
        mixer4.gain(1,0.7);
      }else{
        mixer4.gain(0,1);
        mixer4.gain(1,0);
      }

      if(switch4_state){ 
        /*active hardcore white noise*/
        mixer3.gain(0,0.2);
        mixer3.gain(1,1);
        mixer3.gain(2,1);
        mixer3.gain(3,1);
      }else{
        mixer3.gain(0,1);
        mixer3.gain(1,0);
        mixer3.gain(2,0);
        mixer3.gain(3,0);
      }

      if(switch5_state){
        /*active polarity inverted*/
        mixer5.gain(0,1);
        mixer5.gain(1,0);
      }else{
        mixer5.gain(0,0);
        mixer5.gain(1,1);
      }
    }
  }
  else{
    /*No signal detected, teensy passthrough mode*/
    signalPassthrough();
  }

    prev_switch1_state         = switch1_state;
    prev_switch2_state         = switch2_state;
    prev_switch3_state         = switch3_state;
    prev_switch4_state         = switch4_state;
    prev_switch5_state         = switch5_state;
    prev_state_signal_detected = signal_detected;

}
