// LedMatrixPatterns.h

#ifndef LED_MATRIX_PATTERNS_H
#define LED_MATRIX_PATTERNS_H

#include <Arduino.h>
#include <LedController.hpp>

extern byte smile[8];
extern byte ex[8];
extern byte arrow[8];
extern byte null[8];
extern byte n1[8];
extern byte n2[8];
extern byte n3[8];
extern byte n4[8];
extern byte n5[8];

void loadingAnimation(LedController &lc);
void writeMatrix(LedController &lc, byte bname[8]);
void writeMatrixMirror(LedController &lc, byte bname[8]);
void rotateMatrix90Degrees(byte inputMatrix[8], byte outputMatrix[8]);

#endif // LED_MATRIX_PATTERNS_H