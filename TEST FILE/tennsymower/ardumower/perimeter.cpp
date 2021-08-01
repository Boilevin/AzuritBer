#include "perimeter.h"
#include <Arduino.h>
#include <limits.h>
//#include "adcman.h"
#include "config.h"



PerimeterClass Perimeter;

/*
  // developer test to be activated in mower.cpp:
  #ifdef USE_DEVELOPER_TEST
  // more motor driver friendly signal (receiver)
  int8_t sigcode_norm[] = {1, -1, 0, 0, 0, 1, -1, 0, 0, 0, -1, 1, 0, 0, 0, 1, -1, 0, 0, 0};
  #else
  // http://grauonline.de/alexwww/ardumower/filter/filter.html
  // "pseudonoise4_pw" signal
  // if using reconstructed sender signal, use this
  #if defined (SIGCODE_1) //for station area 1
*/

int8_t sigcode_norm1[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };
int8_t sigcode_diff1[] = { 1, 0, -1, 0, 1, -1, 1, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, -1 };


int8_t sigcode_norm2[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };
int8_t sigcode_diff2[] = { 1, 0, -1, 0, 1, -1, 1, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, -1 };

//int8_t sigcode_norm2[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1 };
//int8_t sigcode_diff2[] = { 0, 0, -1,  0, 1, -1, 1, -1,  0, 1, -1, 1, 0, -1,  0, 1, -1,  0, 1, -1,  0, 1, 0, -1, 1, 0, -1,  0,  0, 1, -1,  0, 1, -1, 1, -1,  0, 1, 0, -1, 1, 0, -1,  0, 1, 0, -1, 1 };

int8_t sigcode_norm3[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };
int8_t sigcode_diff3[] = { 1, 0, -1, 0, 1, -1, 1, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, -1 };
/*
int8_t sigcode_norm3[] = { 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1,
                           -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1
                         };
int8_t sigcode_diff3[] = { 0, 0, -1, 0, 0, 1, -1, 0, 1, -1, 1, -1, 0, 1, 0, -1, 1, 0, -1, 0, 1, 0, -1, 1, 0, -1, 1, -1, 1, -1, 0, 1, -1, 0, 1, -1, 1, 0, -1, 1, -1, 1, -1, 1, -1, 1, 0, -1, 0, 1, 0, -1, 0, 1, 0, -1, 1, -1, 1, 0, -1, 0, 1, -1,
                           0, 1, 0, -1, 1, 0, -1, 0, 1, -1, 0, 1, -1, 1, -1, 1, 0, -1, 1, -1, 0, 1, -1, 0, 1, 0, -1, 1, -1, 0, 1, 0, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 0, -1, 0, 1, -1, 0, 1, -1, 1, -1, 0, 1, -1, 0, 1, 0, -1, 1, 0, -1, 0, 1
                         };
*/
/*
  //int8_t sigcode_norm[]        = { 1,1,-1,-1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,1,1,-1 };
  // "pseudonoise4_pw" signal (differential)
  // if using the coil differential signal, use this
  //int8_t sigcode_diff[]        = { 1,0,-1, 0,1,-1,1,-1, 0,1,-1,1,0,-1, 0,1,-1, 0,1,-1, 0,1,0,-1 };

  #endif
*/


int8_t sigcode_norm[128];
int8_t sigcode_diff[128];
int16_t sigcode_size;

PerimeterClass::PerimeterClass() {
  //useDifferentialPerimeterSignal = true;
  swapCoilPolarityLeft = false;
  swapCoilPolarityRight = false;
  //read2Coil=true;
  timedOutIfBelowSmag = 50;
  timeOutSecIfNotInside = 15;
  callCounter = 0;
  mag[0] = mag[1] = 0;
  smoothMag[0] = smoothMag[1] = 0;
  filterQuality[0] = filterQuality[1] = 0;
  signalCounter[0] = signalCounter[1] = 0;
  lastInsideTime[0] = lastInsideTime[1] = 0;
}


void PerimeterClass::changeArea(byte areaInMowing) {
  Console.print("Change to Area : ");
  Console.println(areaInMowing);
  for (int uu = 0 ; uu <= 128; uu++) { //clear the area
    sigcode_norm[uu] = 0;
    sigcode_diff[uu] = 0;
  }
  switch (areaInMowing) {

    case 1:
      sigcode_size = sizeof sigcode_norm1;
      for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode_norm1[uu];
        sigcode_diff[uu] = sigcode_diff1[uu];
      }

      break;
    case 2:
      sigcode_size = sizeof sigcode_norm2;
      for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode_norm2[uu];
        sigcode_diff[uu] = sigcode_diff2[uu];
      }

      break;
    case 3:
      sigcode_size = sizeof sigcode_norm3;
      for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode_norm3[uu];
        sigcode_diff[uu] = sigcode_diff3[uu];
      }

      break;

  }

  Console.print("New sigcode in use  : ");

  for (int uu = 0 ; uu <= (sigcode_size - 1); uu++) {
    Console.print(sigcode_norm[uu]);
    Console.print(",");
  }
  Console.println();

}


void PerimeterClass::begin(byte idx0Pin, byte idx1Pin) {
  idxPin[0] = idx0Pin;
  idxPin[1] = idx1Pin;
/*
  switch (ADCMan.sampleRate) {
    case SRATE_9615: subSample = 1; break;
    case SRATE_19231: subSample = 2; break;
    case SRATE_38462: subSample = 4; break;
  }
*/
  // use max. 1024 samples and multiple of signalsize
  int adcSampleCount = sigcode_size * subSample;
  pinMode(idx0Pin, INPUT);
  pinMode(idx1Pin, INPUT);

  /*
  ADCMan.setupChannel(idx0Pin,  2 * adcSampleCount, true);
  ADCMan.setupChannel(idx1Pin,  2 * adcSampleCount, true);
*/


 
  Console.print(F("matchSignal size="));
  Console.println(sigcode_size);
  Console.print(F("subSample="));
  Console.println((int)subSample);
  Console.println(F("capture size="));
  //Console.println(ADCMan.getSampleCount(idx0Pin));
}

void PerimeterClass::speedTest() {
  int loops = 0;
  unsigned long endTime = millis() + 1000;
  while (millis() < endTime) {
    matchedFilter(0);
    loops++;
  }
  Console.print("Read in 1 sec ");
  Console.println(loops);

}

const int8_t* PerimeterClass::getRawSignalSample(byte idx) {
  //return rawSignalSample[idx];
  return NULL;
}

void PerimeterClass::run() {
  byte maxindex;
  //bb 2 coil read
  if (read2Coil) maxindex = 2;
  else maxindex = 1;

  for (int idx = 0; idx < maxindex; idx++) {
/*
    if (ADCMan.isConvComplete(idxPin[idx])) {
      matchedFilter(idx);
    }
    */
  }
}

int PerimeterClass::getMagnitude(byte idx) {
  return mag[idx];
}

int PerimeterClass::getSmoothMagnitude(byte idx) {
  return smoothMag[idx];
}

void PerimeterClass::printADCMinMax(int8_t *samples) {
  int8_t vmax = SCHAR_MIN;
  int8_t vmin = SCHAR_MAX;
  /*
  for (byte i = 0; i < ADCMan.getSampleCount(idxPin[0]); i++) {
    vmax = max(vmax, samples[i]);
    vmin = min(vmin, samples[i]);
  }
  */
  Console.print(F("perimter min,max="));
  Console.print((int)vmin);
  Console.print(F(","));
  Console.println((int)vmax);
}

// perimeter V2 uses a digital matched filter
void PerimeterClass::matchedFilter(byte idx) {
  /*
  int16_t sampleCount = ADCMan.getSampleCount(idxPin[0]);
  int8_t *samples = ADCMan.getSamples(idxPin[idx]);
  */
  if (callCounter == 100) {
    // statistics only
    callCounter = 0;
    signalMin[idx] = 9999;
    signalMax[idx] = -9999;
    signalAvg[idx] = 0;
    /*
    for (int i = 0; i < sampleCount; i++) {
      int8_t v = samples[i];
      
      //Console.print(v);
      //Console.print(",");
      
      signalAvg[idx] += v;
      signalMin[idx] = min(signalMin[idx], v);
      signalMax[idx] = max(signalMax[idx], v);
    }
    */
    //Console.println(" ");
    signalAvg[idx] = ((double)signalAvg[idx]) / ((double)(sampleCount));
  }
  // magnitude for tracking (fast but inaccurate)

  //int16_t sigcode_size = sizeof sigcode_norm;

  int8_t *sigcode = sigcode_norm;
  //if (useDifferentialPerimeterSignal) sigcode = sigcode_diff;
  sigcode = sigcode_diff;
  mag[idx] = corrFilter(sigcode, subSample, sigcode_size, samples, sampleCount - sigcode_size * subSample, filterQuality[idx]);

  if ((idx == 0) && swapCoilPolarityLeft) mag[idx] *= -1;
  if ((idx == 1) && swapCoilPolarityRight) mag[idx] *= -1;
  // smoothed magnitude used for signal-off detection change from 1 % to 5 % for faster detection and possible use on center big area to avoid in/out transition
  smoothMag[idx] = 0.95 * smoothMag[idx] + 0.05 * ((float)abs(mag[idx]));
  //smoothMag[idx] = 0.99 * smoothMag[idx] + 0.01 * ((float)abs(mag[idx]));
  
  // perimeter inside/outside detection
  if (mag[idx] > 0) {
    signalCounter[idx] = min(signalCounter[idx] + 1, 5);
  } else {
    signalCounter[idx] = max(signalCounter[idx] - 1, -5);
  }
  if (mag[idx] < 0) {
    lastInsideTime[idx] = millis();
  }

 // ADCMan.restartConv(idxPin[idx]);
  if (idx == 0) callCounter++;
}

void PerimeterClass::resetTimedOut() {
  lastInsideTime[0] = millis();
  lastInsideTime[1] = millis();
}

int16_t PerimeterClass::getSignalMin(byte idx) {
  return signalMin[idx];
}

int16_t PerimeterClass::getSignalMax(byte idx) {
  return signalMax[idx];
}

int16_t PerimeterClass::getSignalAvg(byte idx) {
  return signalAvg[idx];
}


float PerimeterClass::getFilterQuality(byte idx) {
  return filterQuality[idx];
}

boolean PerimeterClass::isInside() {

  return (isInside(IDX_LEFT));
  //return (isInside(IDX_LEFT) && isInside(IDX_RIGHT));
}

boolean PerimeterClass::isInside(byte idx) {
  if (abs(mag[idx]) > 600) {
    // Large signal, the in/out detection is reliable.
    // Using mag yields very fast in/out transition reporting.
    return (mag[idx] < 0);
  } else {
    // Low signal, use filtered value for increased reliability
    return (signalCounter[idx] < 0);
  }
}

boolean PerimeterClass::signalTimedOut() {

  return (signalTimedOut(IDX_LEFT) && signalTimedOut(IDX_RIGHT));
}


boolean PerimeterClass::signalTimedOut(byte idx) {
  if (getSmoothMagnitude(idx) < timedOutIfBelowSmag) return true;
  if (millis() - lastInsideTime[idx] > timeOutSecIfNotInside * 1000) return true;
  return false;
}


// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// H[] holds the double sided filter coeffs, M = H.length (number of points in FIR)
// subsample is the number of times for each filter coeff to repeat
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data

int16_t PerimeterClass::corrFilter(int8_t *H, int8_t subsample, int16_t M, int8_t *ip, int16_t nPts, float & quality) {
  /*
    Console.print("H:");
    Console.print(*H);
    Console.print(" M:");
    Console.print(M);
    Console.print(" *ip:");
    Console.print(*ip);
    Console.print(" nPts:");
    Console.print(nPts);
    Console.println();
  */
  int16_t sumMax = 0; // max correlation sum
  int16_t sumMin = 0; // min correlation sum
  int16_t Ms = M * subsample; // number of filter coeffs including subsampling

  // compute sum of absolute filter coeffs
  int16_t Hsum = 0;
  for (int16_t i = 0; i < M; i++) Hsum += abs(H[i]);
  Hsum *= subsample;

  // compute correlation
  // for each input value
  for (int16_t j = 0; j < nPts; j++)
  {
    int16_t sum = 0;
    int8_t *Hi = H;
    int8_t ss = 0;
    int8_t *ipi = ip;
    // for each filter coeffs
    for (int16_t i = 0; i < Ms; i++)
    {
      sum += ((int16_t)(*Hi)) * ((int16_t)(*ipi));
      ss++;
      if (ss == subsample) {
        ss = 0;
        Hi++; // next filter coeffs
      }
      ipi++;
    }
    if (sum > sumMax) sumMax = sum;
    if (sum < sumMin) sumMin = sum;
    ip++;
  }
  // normalize to 4095
  sumMin = ((float)sumMin) / ((float)(Hsum * 127)) * 4095.0;
  sumMax = ((float)sumMax) / ((float)(Hsum * 127)) * 4095.0;

  // compute ratio min/max
  if (sumMax > -sumMin) {
    quality = ((float)sumMax) / ((float) - sumMin);
    return sumMax;
  } else {
    quality = ((float) - sumMin) / ((float)sumMax);
    return sumMin;
  }
}
