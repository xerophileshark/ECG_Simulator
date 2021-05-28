/*
   File name: ecg_due_3.ino
   Descriptin: This sketch uses Arduino Due DAC to output the ecg data. The method applied here is discussed in M. Nasor's paper.
   Notes:
    1. The sampling frequency is set to 1000 Hz using channel 0 of TC1.
    2. The output voltage range is "v" and need an output op-amp circuit with a gain of 0.001 to come into "mv" range.
    3. C++ math.h library's  abs() function supportes double type variables as input.
*/

#include <math.h>

#define SAMPLE_FREQ 1000 // Hz
#define PWAVE_AMP 0.24 // Volt
#define QWAVE_AMP 0.025 // Volt
#define RWAVE_AMP 1.6 // Volt
#define SWAVE_AMP 0.25 // Volt // Bias
#define TWAVE_AMP 0.35 // Volt

//Define global variables:
volatile bool outputflag;
volatile bool noiseflag;
volatile int state;
volatile int HR; //Heart rate (BPM)
volatile double T_RtoR;
volatile double t; //Time holder variable
volatile double delta_t; //delta_t = 0.001s

//Define interval and duration holder variables:
volatile double P_duration;
volatile double PtoQ_interval;
volatile double QRS_duration;
volatile double Q_duration;
volatile double R_duration;
volatile double S_duration;
volatile double StoT_interval;
volatile double T_duration;
volatile double total_duration;

//Function defenitions:
void calc_intervals(void);
double triangle(double t_now, double t_start, double duration, double amplitude); //All units are in second.
double second_order(double t_now, double t_start, double duration, double amplitude); //All units are in second.

//Timer Interrupt Service: TC1 ch 0:
void TC3_Handler()
{
  TC_GetStatus(TC1, 0); // ???
  if (outputflag == true) {
    if (t < ((HR <= 162) ? total_duration : T_RtoR)) { // for HR = 162, "T_RtoR = 0.3704" and "total_duration = 0.3703".
      // Amplitude mapping and output generate
      double output =/* Bias */SWAVE_AMP + /* P wave: */second_order(t, 0, P_duration, PWAVE_AMP) - /* Q wave: */triangle(t, P_duration + PtoQ_interval, Q_duration, QWAVE_AMP) + /* R wave: */triangle(t, P_duration + PtoQ_interval + Q_duration, R_duration, RWAVE_AMP) - /* S wave */triangle(t, P_duration + PtoQ_interval + Q_duration + R_duration, S_duration, SWAVE_AMP) + /* T wave: */second_order(t, P_duration + PtoQ_interval + QRS_duration + StoT_interval, T_duration, TWAVE_AMP);
      int DAC_output = (int)(output * 1862); // maximum DAC_output = 3444 for (0.25 + 1.6) = 1.85 volts.
      analogWrite(DAC0, DAC_output);
      //Serial.println(output); //OUTPUT
      t += delta_t;
    }
    else { // This scope needs to be trimmed a little bit
      if (t <= T_RtoR) {
        int DAC_output = 0 + 465; // 0.25 * 1862 = 465.5
        analogWrite(DAC0, DAC_output);
        //Serial.println(0); //OUTPUT
        t += delta_t;
      } else {
        t = 0;
      }
    }
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

void setup() {
  analogWriteResolution(12); // The default resolution is 8 bit.

  //Variable initiation:
  outputflag = false;
  noiseflag = false;
  state = 1;
  HR = 0;
  T_RtoR = 0;
  t = 0;
  delta_t = (double)(1.0 / SAMPLE_FREQ);

  P_duration = 0;
  PtoQ_interval = 0;
  QRS_duration = 0;
  Q_duration = 0;
  R_duration = 0;
  S_duration = 0;
  StoT_interval = 0;
  T_duration = 0;
  total_duration = 0;

  //Start timer:
  startTimer(TC1, 0, TC3_IRQn, SAMPLE_FREQ); //TC1 channel 0, the IRQ for that channel and the desired frequency (=1000HZ)

  //Initiate serial interface:
  Serial.begin(115200);
}//end setup

void loop() {
  if (outputflag == false) {
    Serial.println(">> Enter desired Heart Rate:");
    delay(100);
    while (true) {
      if (Serial.available()) {
        HR = (int) Serial.readString().toInt();
        Serial.print("[OK] Heart rate "); Serial.print(HR); Serial.println(" is selected.");
        T_RtoR = (double)(60.0 / (double)HR);
        Serial.print("[OK] T_RtoR = "); Serial.print(T_RtoR); Serial.println(" .");
        Serial.print(">> Calculating intervals ... ");
        calc_intervals();
        Serial.println("[OK]");
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
        Serial.println("[OK] Generating ECG signal started!");
        t = 0;
        outputflag = true;
        break; // Start to generate output
      } else {
        //Serial.println(".");
        delay(100);
      }
    }
  }
}

//Functions implementation:
void calc_intervals()
{
  P_duration = 0.37 * sqrt(T_RtoR) - 0.22 * T_RtoR - 0.06;
  PtoQ_interval = 0.33 * sqrt(T_RtoR) - 0.18 * T_RtoR - 0.08;
  QRS_duration = 0.25 * sqrt(T_RtoR) - 0.16 * T_RtoR - 0.02;
  Q_duration = QRS_duration * 0.23; // Obtained experimentally
  R_duration = QRS_duration * 0.42; // Obtained experimentally
  S_duration = QRS_duration * 0.35; // Obtained experimentally
  StoT_interval = -0.09 * sqrt(T_RtoR) + 0.13 * T_RtoR + 0.04;
  T_duration = 1.06 * sqrt(T_RtoR) - 0.51 * T_RtoR - 0.33;
  total_duration = P_duration + PtoQ_interval + QRS_duration + StoT_interval + T_duration;
}

double triangle(double t_now, double t_start, double duration, double amplitude)
{
  double output = 0.0;
  double a = duration * 0.5;
  double alpha = t_start + 0.5 * duration;
  double Gain = amplitude / a;
  //Check if t_now is in a suitable range:
  if (t_now >= t_start && t_now <= t_start + duration ) {
    output = -abs(t_now - alpha) + a;
  }
  return (output * Gain);
}

double second_order(double t_now, double t_start, double duration, double amplitude)
{ // y = Gain * (- (t - alpha)^2 + a ) ; alpha = t_start+(duration/2) , a = (td^2)/4 , Gain = amplitude[a_desired]/a ; t = t_now .
  double output = 0.0;
  double a = duration * duration * 0.25;
  double alpha = t_start + 0.5 * duration;
  double Gain = amplitude / a;
  //Check if t_now is in a suitable range:
  if (t_now >= t_start && t_now <= t_start + duration ) {
    output = -((t_now - alpha) * (t_now - alpha)) + a;
  }
  return (output * Gain);
}
