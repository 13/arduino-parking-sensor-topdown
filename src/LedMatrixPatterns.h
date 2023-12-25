// LedMatrixPatterns.h

#ifndef LED_MATRIX_PATTERNS_H
#define LED_MATRIX_PATTERNS_H

#include <Arduino.h>
#include <LedController.hpp>

extern byte arrow[8];
extern byte ex[8];
extern byte null[8];
extern byte num1[8];
extern byte num2[8];
extern byte num3[8];
extern byte num4[8];
extern byte num5[8];
extern byte smile[8];
extern byte timedout[8];

void initDisplay(LedController &lc, unsigned int intensity);
void loadingAnimation(LedController &lc);
void writeMatrix(LedController &lc, byte bname[8]);
void writeMatrixInv(LedController &lc, byte bname[8]);

#endif // LED_MATRIX_PATTERNS_H