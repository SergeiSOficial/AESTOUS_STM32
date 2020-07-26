#include "myPid.h"
#include "math.h"
#include <stdint.h>
#include <stdlib.h>
#include "tim.h"
/*
PID PERIOD TIME IN MAIN FILE (CONST VALUE IN HEADER, BASING ON TIMER INTERRUPT)!
PID KOEFFICENT IN THIS FILE IN PID REGULATOR FUNCTION
*/



const float termistor[43][3] = {
    {-55.0, 96.3, 7.4},
    {-50.0, 67.01, 7.2},
    {-45.0, 47.17, 6.9},
    {-40.0, 33.65, 6.7},
    {-35.0, 24.26, 6.4},
    {-30.0, 17.7, 6.2},
    {-25.0, 13.04, 6.0},
    {-20.0, 9.707, 5.8},
    {-15.0, 7.293, 5.6},
    {-10.0, 5.533, 5.5},
    {-5.0, 4.232, 5.3},
    {0.0, 3.265, 5.1},
    {5.0, 2.539, 5.0},
    {10.0, 1.99, 4.8},
    {15.0, 1.571, 4.7},
    {20.0, 1.249, 4.5},
    {25.0, 1.0000, 4.4},
    {30.0, 0.8057, 4.3},
    {35.0, 0.6531, 4.1},
    {40.0, 0.5327, 4.0},
    {45.0, 0.4369, 3.9},
    {50.0, 0.3603, 3.8},
    {55.0, 0.2986, 3.7},
    {60.0, 0.2488, 3.6},
    {65.0, 0.2083, 3.5},
    {70.0, 0.1752, 3.4},
    {75.0, 0.1481, 3.3},
    {80.0, 0.1258, 3.2},
    {85.0, 0.1072, 3.2},
    {90.0, 0.09177, 3.1},
    {95.0, 0.07885, 3.0},
    {100.0, 0.068, 2.9},
    {105.0, 0.05886, 2.9},
    {110.0, 0.05112, 2.8},
    {115.0, 0.04454, 2.7},
    {120.0, 0.03893, 2.6},
    {125.0, 0.03417, 2.6},
    {130.0, 0.03009, 2.5},
    {135.0, 0.02654, 2.5},
    {140.0, 0.02348, 2.4},
    {145.0, 0.02083, 2.4},
    {150.0, 0.01853, 2.3},
    {155.0, 0.01653, 2.3}

};

// extern uint16_t SDADC1data3[3000];
// extern uint16_t SDADC2data2[2000];
// extern uint16_t SDADC3data3[3000];

float NTC_read(unsigned char termPosition)
{

  uint32_t all_summ = 0;
  float result = 0;
  int b = 0;

  switch (termPosition)
  {
  case 1:
	while (b<1000) {
    result += ADC_GetChannel0();
    b++;
	}
	result = result / 1000;

    break;
  case 2:
    result = ADC_GetChannel1();
    break;
  case 3:
    result = ADC_GetChannel5();
    break;
  case 4:
    result = ADC_GetTemp();
    break;
  case 5:
    result = ADC_GetVref();
    break;
  default:
    result = ADC_GetChannel0();
    break;
  }

  //float resistance_ntc = 4095 - result / result;
  //protection for 0 division
  float resistance_ntc = 0.0;
  if (result != 4095)
  {
	  resistance_ntc = ((result * 5.1) / (4095 - result)) * 1.0 + 0.0;
  }


  float b25100RealCoef = resistance_ntc / 10.0;
  int i = 0;
  while (termistor[i][1] > b25100RealCoef)
  {
    i++;
  }

  float Tx = termistor[i - 1][0];
  float a = termistor[i - 1][2];
  float RTx = termistor[i - 1][1] * 10;
  float Rt = resistance_ntc;

  result = (1.00 / (((((log(Rt / RTx)) / (a / 100.00 * (Tx + 273.15) * (Tx + 273.15)))) + 1.00 / (Tx + 273.15)))) - 273.15; //Steinhart–Hart equation
  return result;
}

void freqPower(int power) {
	float compute_freq = 0.0;
	compute_freq = 2000 * powf((1000-(float)power)/90,2.1);

    if (compute_freq < 2000) {
    	compute_freq = 2000;
    }

    if (compute_freq > 300000) {
    	compute_freq = 300000;
    }

	TIM_SetGenFreq((int)compute_freq);
}

char PidBigBlock(float SetTemp)
{

  static float TemperatureBigBlockReal = 0.0;
  static float TemperatureBigBlockSet = 0.0;

  static int first_call = 'y';
  if (SetTemp != TemperatureBigBlockSet)
  {
    first_call = 'y';
    TemperatureBigBlockSet = SetTemp;
  }

  static float vozdeistvie = 0, Integralnoe_proshloe = 0, Kp = 30, Ki = 10, Kd = 3, Oshibka = 0, proshlOhibka = 0; // vozdeystvie_proshloe
  if (first_call == 'y')
  {
    first_call = 'n';
    Integralnoe_proshloe = 0;
    //   vozdeystvie_proshloe = 0;
  }

  TemperatureBigBlockReal = NTC_read(1);
  Oshibka = TemperatureBigBlockSet - TemperatureBigBlockReal; // Proportional error calculating

  vozdeistvie = Ki * Oshibka + Integralnoe_proshloe; // Integral proportional (Fist part of PID) ()
  Integralnoe_proshloe = vozdeistvie;                //
  vozdeistvie += Kp * Oshibka;                       // Proportional part opf PID
  vozdeistvie += Kd * (Oshibka - proshlOhibka);      // Differential part of PID
  proshlOhibka = Oshibka;

  if (vozdeistvie < 0)
    vozdeistvie = 0; // Output LIMITS (Lower SIDE)  max 1000
  if (vozdeistvie >= 1000)
    vozdeistvie = 1000; // Output LIMIT (Upper SIDE)   max 1000

  // vozdeystvie_proshloe = vozdeistvie;                        //

  int OgrINT = 750; // MAX error summ (Integral part)
  int OgrINTo = 0;  // LOW error summ (Integral part)
                    // int VozdPr = vozdeistvie;

  if (Integralnoe_proshloe > OgrINT)
    Integralnoe_proshloe = OgrINT;
  if (Integralnoe_proshloe < OgrINTo)
    Integralnoe_proshloe = OgrINTo;

  freqPower(vozdeistvie);
  //TIM_SetGenFreq(vozdeistvie*48);
  TIM_SetPWMEnCounter(1000*48);

  // if (vozdeystvie_proshloe > VozdPr) vozdeystvie_proshloe = VozdPr;

  // peliter(1, 'n', (int)vozdeistvie);
  // peliter(2, 'n', (int)vozdeistvie);
/*
  float module = 0.0;
  module = TemperatureBigBlockSet - TemperatureBigBlockReal;
  module = abs(module);

  if (module <= 3)
  {
    return 'R';
  }
  else
  {
    return 'N';
  }
  */
}
