// LedMatrixPatterns.cpp

#include "LedMatrixPatterns.h"

byte smile[8] = {
    B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10100101,
    B10011001,
    B01000010,
    B00111100};

byte ex[8] = {
    B10000001,
    B01000010,
    B00100100,
    B00011000,
    B00011000,
    B00100100,
    B01000010,
    B10000001};

byte arrow[8] = {
    B00010000,
    B00101000,
    B01000100,
    B10000010,
    B00010000,
    B00101000,
    B01000100,
    B10000010};

byte null[8] = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000};

byte n1[8] = {
    B00000000,
    B00001000,
    B00011000,
    B00101000,
    B00001000,
    B00001000,
    B00011100,
    B00000000};

byte n2[8] = {
    B00000000,
    B00011000,
    B00100100,
    B00001000,
    B00010000,
    B00100000,
    B00111100,
    B00000000};

byte n3[8] = {
    B00000000,
    B00111100,
    B00000100,
    B00011000,
    B00000100,
    B00100100,
    B00011000,
    B00000000};

byte n4[8] = {
    B00000000,
    B00000100,
    B00001100,
    B00010100,
    B00111100,
    B00000100,
    B00000100,
    B00000000};

byte n5[8] = {
    B00000000,
    B00111100,
    B00100000,
    B00111000,
    B00000100,
    B00100100,
    B00011000,
    B00000000};

void rotateMatrix90Degrees(byte inputMatrix[8], byte outputMatrix[8])
{
  for (int row = 0; row < 8; row++)
  {
    for (int col = 0; col < 8; col++)
    {
      bitWrite(outputMatrix[row], col, bitRead(inputMatrix[7 - col], row));
    }
  }
}

void writeMatrix(LedController &lc, byte bname[8])
{
  for (int i = 0; i <= 7; i++)
  {
    // lc.setColumn(0, i, bname[i]);
    lc.setColumn(0, i, bname[7 - i]);
    delay(1);
  }
}

void writeMatrixMirror(LedController &lc, byte bname[8])
{
  for (int i = 0; i <= 7; i++)
  {
    lc.setRow(0, i, bname[7 - i]);
    delay(1);
  }
}

void loadingAnimation(LedController &lc)
{
  for (int j = 0; j < 4; j++)
  {
    for (int i = 0; i < 8; i++)
    {
      lc.setColumn(0, i, B11111111);
      if (i != 3)
      {
        lc.setColumn(0, 7 - i, B11111111);
      }
      delay(100);
      if (j != 3)
      {
        lc.clearMatrix();
      }
    }
  }
}