/*
   File name: ecg_due.ino
   By: Ali Abedi, aliabediemail@yahoo.com
   This sketch uses Arduino Due DAC to output the ecg data. The method applied here is "Fourier Series method".
   Tested on Arduino Due on Saturday, October 26, 2019.
   NOTES:
    1. In Arduino Due, DAC output range is actually from 0.55 V to 2.75 V only. (https://store.arduino.cc/usa/due -> documentation tab)
    2. Benefitial information about Arduini Due timers: http://ko7m.blogspot.com/2015/01/arduino-due-timers-part-1.html.
    3. All variable names are according to "ECG_Simulation_using_Fourier_Series_From.pdf" and mostly "ECG.pdf" papers.
    4. On Arduino Due <double> variables have 8 bytes (64 bits) precision.
    5. The output unit is "Volt" but it should be converted into "milli Volt" range with an outside OpAmp
	7. Some calculations:
		2L = 1/BPS = 60/BPM => L = 30/BPM .	*** BPM = desired Heart Rate.
		b = 2*L/d . *** d = duration of the wave.
	8. The signal starts from the peak of R wave, so we 
		generate output from t=0 to t=T_RtoR; With this time, we have an array of the periodic heart beat.		
*/

#include <math.h>

#define SAMPLE_FREQ 1000 //Hz
#define AMP_QRSWAV 1.6 // (v)
#define DUR_QRSWAV 0.11 // (sec) 0.06 to 0.12
#define AMP_QWAV 0.25 // (v) [NEGATIVE]
#define DUR_QWAV 0.066 // (sec)
#define INT_QWAV 0.166 // [CONSTANT]
#define AMP_SWAV 0.25 // [NEGATIVE]
#define DUR_SWAV 0.066
#define INT_SWAV 0.09 // [CONSTANT]
#define AMP_PWAV 0.25
#define DUR_PWAV 0.09
#define INT_PWAV 0.16
#define AMP_TWAV 0.35
#define DUR_TWAV 0.142
#define INT_TWAV 0.2
#define AMP_UWAV 0.035
#define DUR_UWAV 0.0476
#define INT_UWAV 0.433 //CONSTANT

#define N 50 // Number of coefficients Cn and Dn

volatile double qrs_Cn[N] = {};
volatile double qrs_Dn[N] = {};
volatile double q_Cn[N] = {};
volatile double q_Dn[N] = {};
volatile double s_Cn[N] = {};
volatile double s_Dn[N] = {};
volatile double p_Cn[N] = {};
volatile double p_Dn[N] = {};
volatile bool l = false;

volatile int HR; // HeartRate (BPM)
/* \/\/ NEWLY ADDED \/\/ */
volatile double T_RtoR; // R-R interval (sec)
volatile double t; //time :: t = 0.0 to T_RtoR
/* ^^^ NEWLY ADDED ^^^*/
volatile bool outputflag;
volatile bool noiseflag;

double qrs_wave(double t, double hr);
double q_wave(double t, double hr);
double s_wave(double t, double hr);
double p_wave(double t, double hr);
void generate_coefs(double hr);

//Timer Interrupt Service: TC1 ch 0:
void TC3_Handler()
{
  TC_GetStatus(TC1, 0); // ???
  if (outputflag) {
    t += delta_t;
    double output = AMP_QWAV + AMP_SWAV + qrs_wave(t, HR) + q_wave(t + INT_QWAV, HR) + s_wave(t - INT_SWAV, HR) + p_wave(t + INT_PWAV, HR);
    analogWrite(DAC0, (int)(output * 1000));
  }
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK / 128 / frequency; //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc / 2); //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}

void setup()
{
  analogWriteResolution(12); // The default resolution is 8 bit.

  // Initialize variables:
  t = 0;
  delta_t = 1.0 / SAMPLE_FREQ;
  HR = 0;
  outputflag = false;

  // Start timer interrupt service:
  startTimer(TC1, 0, TC3_IRQn, SAMPLE_FREQ); //TC1 channel 0, the IRQ for that channel and the desired frequency (=1000HZ)

  // Initialize Serial Interface:
  Serial.begin(115200);
}

void loop()
{
  if (outputflag == false) {
    Serial.println(">> Enter desired heart rate:");
    delay(250);
    while (true) {
      if (Serial.available()) {
        HR = Serial.readString().toInt();
        T_RtoR = (double)(60.0 / (double)HR);
        Serial.println(""); Serial.print("[OK] "); Serial.print(HR); Serial.println(" is selected as HR.");
        Serial.println(">> Do you want to add noise? (y/n)");
        while (!Serial.available()) {}
        String d = Serial.readString();
        if (d.indexOf("y") >= 0) {
          noiseflag = true;
          Serial.println("[OK] Noise generation: True!");
        } else {
          noiseflag = false;
          Serial.println("[OK] Noise generation: False!");
        }
        outputflag = true;
        Serial.println("[OK] Signal generation started!");
        break;
      } else {
        Serial.print(".");
        delay(200);
      }
    }
  }
}

//Triangular Wave Functions:

double qrs_wave(double t, double hr) { // At origin (t = 0)
  double qrs_b = 2 * /*qrs_L*/(30 / hr) / DUR_QRSWAV;
  double a0 = 0.5 * AMP_QRSWAV / qrs_b;
  double res = a0;
  int i;
  // Generate the QRS signal:
  for (i = 0; i < N; i++) {
    res = res + qrs_Cn[i] * cos(qrs_Dn[i] * t);
  }
  return (res + a0);
}

double q_wave(double t, double hr) {
  double q_b = 2 * /*q_L*/(30 / hr) / DUR_QWAV;
  double a0 = 0.5 * AMP_QWAV / q_b;
  double res = a0;
  int i;
  // Generate the Q signal:
  for (i = 0; i < N; i++) {
    res = res + q_Cn[i] * cos(q_Dn[i] * t);
  }
  return -(res + a0);
}

double s_wave(double t, double hr) {
  if (t <= 0) {
    return 0;
  }
  double s_b = 2 * /*s_L*/(30 / hr) / DUR_SWAV;
  double a0 = 0.5 * AMP_SWAV / s_b;
  double res = a0;
  int i;
  // Generate the S signal:
  for (i = 0; i < N; i++) {
    res = res + s_Cn[i] * cos(s_Dn[i] * t);
  }
  return -(res + a0);
}

// Sinusoidal Wave Functions:

double p_wave(double t, double hr) {
  double p_b = 2 * /*p_L*/(30 / hr) / DUR_PWAV;
  double a0 = 2 / p_b / PI;
  double res = a0;
  int i;
  // Generate the P signal:
  for (i = 0; i < N; i++) {
    res = res + p_Cn[i] * cos(p_Dn[i] * t);
  }
  return (res + a0);
}

// Coefficient generator function:

void generate_coefs(double hr) {
  int i;
  // QRS Wave coeffs:
  double qrs_b = 0;
  double qrs_L = 0;
  qrs_L = 30 / hr;
  qrs_b = 2 * qrs_L / DUR_QRSWAV;
  for (i = 0; i < N; i++) {
    qrs_Cn[i] = (2 * AMP_QRSWAV * qrs_b / ((i + 1) * (i + 1) * PI * PI)) * (1 - cos((i + 1) * PI / qrs_b));
    qrs_Dn[i] = (i + 1) * PI / qrs_L;
  }

  // Q Wave coeffs:
  double q_b = 0;
  double q_L = 0;
  q_L = 30 / hr;
  q_b = 2 * q_L / DUR_QWAV;
  for (i = 0; i < N; i++) {
    q_Cn[i] = (2 * AMP_QWAV * q_b / ((i + 1) * (i + 1) * PI * PI)) * (1 - cos((i + 1) * PI / q_b));
    q_Dn[i] = (i + 1) * PI / q_L;
  }

  // S Wave coeffs:
  double s_b = 0;
  double s_L = 0;
  s_L = 30 / hr;
  s_b = 2 * s_L / DUR_SWAV;
  for (i = 0; i < N; i++) {
    s_Cn[i] = (2 * AMP_SWAV * s_b / ((i + 1) * (i + 1) * PI * PI)) * (1 - cos((i + 1) * PI / s_b));
    s_Dn[i] = (i + 1) * PI / s_L;
  }

  // P Wave coeffs:
  double p_b = 0;
  double p_L = 0;
  p_L = 30 / hr;
  p_b = 2 * p_L / DUR_PWAV;
  for (i = 0; i < N; i++) {
    p_Cn[i] = (4 * p_b / ((p_b - 2 * (i + 1)) * (p_b + 2 * (i + 1)) * PI) * cos((i + 1) * PI / p_b));
    p_Dn[i] = (i + 1) * PI / p_L;
  }

}

// Output generator function:

void generate_outputArray() {
  ;
}
