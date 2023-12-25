// LedMatrixPatterns.cpp

#include "LedMatrixPatterns.h"

byte arrow[8] = {
    B00010000,
    B00101000,
    B01000100,
    B10000010,
    B00010000,
    B00101000,
    B01000100,
    B10000010};

byte ex[8] = {
    B10000001,
    B01000010,
    B00100100,
    B00001000,
    B00010000,
    B00100100,
    B01000010,
    B10000001};

byte null[8] = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00000000};

byte num1[8] = {
    B00000000,
    B00001000,
    B00011000,
    B00101000,
    B00001000,
    B00001000,
    B00011100,
    B00000000};

byte num2[8] = {
    B00000000,
    B00011000,
    B00100100,
    B00001000,
    B00010000,
    B00100000,
    B00111100,
    B00000000};

byte num3[8] = {
    B00000000,
    B00111100,
    B00000100,
    B00011000,
    B00000100,
    B00100100,
    B00011000,
    B00000000};

byte num4[8] = {
    B00000000,
    B00000100,
    B00001100,
    B00010100,
    B00111100,
    B00000100,
    B00000100,
    B00000000};

byte num5[8] = {
    B00000000,
    B00111100,
    B00100000,
    B00111000,
    B00000100,
    B00100100,
    B00011000,
    B00000000};

byte smile[8] = {
    B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10100101,
    B10011001,
    B01000010,
    B00111100};

byte timedout[8] = {
    B11111111,
    B11111111,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000,
    B00011000};

void initDisplay(LedController &lc, unsigned int intensity)
{
  loadingAnimation(lc);
  lc.setIntensity(intensity);
}

void loadingAnimation(LedController &lc)
{
  lc.clearMatrix();
  lc.setIntensity(0);
  for (int j = 0; j < 4; j++)
  {
    for (int i = 0; i < 8; i++)
    {
      lc.setColumn(0, i, B11111111);
      delay(1);
      if (i != 3)
      {
        lc.setColumn(0, 7 - i, B11111111);
        delay(1);
      }
      delay(80);
      if (j != 3)
      {
        lc.clearMatrix();
      }
    }
  }
  lc.clearMatrix();
}

/*
  ||||
...12...
.9....3.
....6...
........
--------
|MX7219|
--------
  ||||
*/
void writeMatrix(LedController &lc, byte bname[8])
{
  for (int i = 0; i <= 7; i++)
  {
    lc.setColumn(0, i, bname[7 - i]);
    delay(1);
  }
}

void writeMatrixInv(LedController &lc, byte bname[8])
{
  byte mirrored[8];

  for (int i = 0; i < 8; i++)
  {
    mirrored[i] = (bname[i] & 0x01) << 7;
    mirrored[i] |= (bname[i] & 0x02) << 5;
    mirrored[i] |= (bname[i] & 0x04) << 3;
    mirrored[i] |= (bname[i] & 0x08) << 1;
    mirrored[i] |= (bname[i] & 0x10) >> 1;
    mirrored[i] |= (bname[i] & 0x20) >> 3;
    mirrored[i] |= (bname[i] & 0x40) >> 5;
    mirrored[i] |= (bname[i] & 0x80) >> 7;
  }
  for (int col = 0; col < 8; col++)
  {
    lc.setColumn(0, col, mirrored[col]);
    delay(1);
  }
}