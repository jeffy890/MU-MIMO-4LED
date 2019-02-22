//4x4 MIMO Program
//Arduino Mega + DAC7614
//TPA601 Photo Transistor
//written by Kensuke Kobayashi
//starts from 2019/2/8


#include <SPI.h>
#define LDAC  8 //dac on
//SS(slave select) : 53
//MOSI(data) : 51
//SCK(clock) : 52

int i, j;  //for repeating anarogRead
long H[4][4];  //4x4 MIMO H channel
float iH[4][4]; //4x4 Inverce H channel
int mtype[16][4] = {{0,0,0,0}, {0,0,0,1}, {0,0,1,0}, {0,0,1,1},
{0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1},
{1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1},
{1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1}}; // all type of m and data

int backlight[4]; //backlights
int maximumRec[4];
int minimumRec[4];
int midRec[4];
int midRec2[4];
int Recmaxima[4];
int Recminima[4];

//for Mseq function
int leng = 512;  //2^9
boolean m[512];  //for storing Mseq
boolean m2[512];
boolean m3[512];
boolean m4[512];

//data for dac
float dataA, dataB, dataC, dataD;
float data[64];
long sendE[64];
long sendA, sendB, sendC, sendD;
float Dmax = 0;
float Dmin = 0;
long maxD, minD;
int dacmin = 0;
int dacmax = 2040;

//Received data
int RecData[512];
int RecData2[512];

int acRec[4];

int Rec2max = 0;
int Rec2min = 500;

//for measuring time
long t, t2;

//for measuring error
float error;

int bat[4] = {0, 0, 0, 0};

int upperlevel[4];
int lowerlevel[4];

int outputP = 1400;

void setup() {
  pinMode(LDAC, OUTPUT) ;
  digitalWrite(LDAC, LOW);
  SPI.begin() ;
  SPI.setBitOrder(MSBFIRST) ;
  SPI.setClockDivider(SPI_CLOCK_DIV128) ;
  SPI.setDataMode(SPI_MODE0) ;
  Serial.begin(9600);
  zeroDAC();
  Mseq();
  mseqShift(10);
  delay(1000);
  //checkArea();
  middle4();

  //run once
  t2 = millis();
  zeroDAC();
  backlight[0] = analogRead(A0);  //measure backlights
  backlight[1] = analogRead(A1);
  backlight[2] = analogRead(A2);
  backlight[3] = analogRead(A3);

  //Measure H channel
  for (i = 0; i < 4; i++) {
      zeroDAC();
      spiSender(i + 1, outputP);
      delay(2);
      H[0][i] = analogRead(A0) - backlight[0];
      H[2][i] = analogRead(A2)- backlight[2];
      H[3][i] = analogRead(A3) - backlight[3];
    }

  zeroDAC();  //set DAC to zero

  inverceH();
  //send and receive
  for(i=0; i <4; i++){
    for(j=0; j<16; j++){
      data[i*16+j] = (iH[i][0] * mtype[j][0] + iH[i][1] * mtype[j][1] + iH[i][2] * mtype[j][2] + iH[i][3] * mtype[j][3]) * 1000000;
      Dmax = max(Dmax, data[i*16+j]);
      Dmin = min(Dmin, data[i*16+j]);
    }
  }

  for(i=0; i <4; i++){
    for(j=0; j<16; j++){
      data[i*16+j] = (data[i*16+j] - Dmin) / (Dmax -Dmin) * outputP;
      sendE[i*16+j] = (long)data[i*16+j];
    }
  }

  for(i=0; i <16; i++){
      spiSender(1, sendE[i]);
      spiSender(2, sendE[i+16*1]);
      spiSender(3, sendE[i+16*2]);
      spiSender(4, sendE[i+16*3]);
      delay(100);
    }

    zeroDAC();

    Serial.println("---------------------------------------------------------------");
    Serial.println("//  H channel and inverceH //");
    for(i=0; i<4; i++){
      for(j=0; j<4; j++){
        Serial.print(H[i][j]);
        Serial.print("\t");
      }
      for(j=0; j<4; j++){
        Serial.print(iH[i][j], 6);
        Serial.print("\t");
      }
      Serial.println();
    }
    Serial.println();

  Serial.println("//  output data  sendE //");
  for(i=0; i<16; i++){
    for(j=0; j<4; j++){
      Serial.print(sendE[j*16+i]);
      Serial.print("\t");
    }
    for(j=0; j<4; j++){
      Serial.print(mtype[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println();

  Serial.println("//  output data  data //");
  for(i=0; i<16; i++){
    for(j=0; j<4; j++){
      Serial.print(data[j*16+i]);
      Serial.print("\t");
    }
    for(j=0; j<4; j++){
      Serial.print(mtype[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }

  Serial.println("---------------------------------------------------------------");
  Serial.println();
  Serial.println();
  Serial.println();

}

/////////////////////////////////////////////////////////////
/// loop
/////////////////////////////////////////////////////////////
void loop() {


}

void spiSender(int ch, int power) {
  digitalWrite(LDAC, HIGH);
  digitalWrite(SS, LOW);

  if (ch == 1) {
    SPI.transfer((power >> 8) | 0x00); //ch a
    SPI.transfer(power & 0xff);
  }
  else if (ch == 2) {
    SPI.transfer((power >> 8) | 0x30); //ch b
    SPI.transfer(power & 0xff);
  }
  else if (ch == 3) {
    SPI.transfer((power >> 8) | 0xc0); //ch c
    SPI.transfer(power & 0xff);
  }
  else if (ch == 4) {
    SPI.transfer((power >> 8) | 0xf0); //ch d
    SPI.transfer(power & 0xff);
  }
  digitalWrite(SS, HIGH);
  digitalWrite(LDAC, LOW);

}

void inverceH() {
  //Serial.println("now in inverceH function");
  t = micros();
  float deno = 0;
  float a = H[0][0];
  float b = H[0][1];
  float c = H[0][2];
  float d = H[1][0];
  float e = H[1][1];
  float f = H[1][2];
  float g = H[2][0];
  float h = H[2][1];
  float i = H[2][2];
  float j = H[3][0];
  float k = H[3][1];
  float l = H[3][2];
  float m = H[0][3];
  float n = H[1][3];
  float o = H[2][3];
  float p = H[3][3];

  deno = f * h * j * m - e * i * j * m - f * g * k * m + d * i * k * m + e * g * l * m - d * h * l * m - c * h * j * n + b * i * j * n + c * g * k * n - a * i * k * n - b * g * l * n + a * h * l * n + c * e * j * o - b * f * j * o - c * d * k * o + a * f + k * o + b * d * l * o - a * e * l * o - c * e * g * p + b * f * g * p + c * d * h * p - a * f * h * p - b * d * i * p + a * e * i * p;
  iH[0][0] = (-i * k * n + h * l * n + f * k * o - e * l * o - f * h * p + e * i * p) / deno;
  iH[0][1] = (i * k * m - h * l * m - c * k * o + b * l * o + c * h * p - b * i * p) / deno;
  iH[0][2] = (-f * k * m + e * l * m + c * k * n - b * l * n - c * e * p + b * f * p) / deno;
  iH[0][3] = (f * h * m - e * i * m - c * h * n + b * i * n + c * e * o - b * f * o) / deno;
  iH[1][0] = (i * j * n - g * l * n - f * j * o + d * l * o + f * g * p - d * i * p) / deno;
  iH[1][1] = (-i * j * m + g * l * m + c * j * o - a * l * o - c * g * p + a * i * p) / deno;
  iH[1][2] = (f * j * m - d * l * m - c * j * n + a * l * n + c * d * p - a * f * p) / deno;
  iH[1][3] = (-f * g * m + d * i * m + c * g * n - a * i * n - c * d * o + a * f * o) / deno;
  iH[2][0] = (-h * j * n + g * k * n + e * j * o - d * k * o - e * g * p + d * h * p) / deno;
  iH[2][1] = (h * j * m - g * k * m - b * j * o + a * k * o + b * g * p - a * h * p) / deno;
  iH[2][2] = (-e * j * m + d * k * m + b * j * n - a * k * n - b * d * p + a * e * p) / deno;
  iH[2][3] = (e * g * m - d * h * m - b * g * n + a * h * n + b * d * o - a * e * o) / deno;
  iH[3][0] = (f * h * j - e * i * j - f * g * k + d * i * k + e * g * l - d * h * l) / deno;
  iH[3][1] = (-c * h * j + b * i * j + c * g * k - a * i * k - b * g * l + a * h * l) / deno;
  iH[3][2] = (c * e * j - b * f * j - c * d * k + a * f * k + b * d * l - a * e * l) / deno;
  iH[3][3] = (-c * e * g + b * f * g + c * d * h - a * f * h - b * d * i + a * e * i) / deno;
  t = micros() - t;
  //Serial.print("time spent for calculating inverceH >>");
  //Serial.println(t);

  //Serial.print("denominator is >> ");
  //Serial.println(deno, 10);

}

void Mseq() {

  int i, j; //for counting

  boolean m9[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};  //for storing data right before outputting
  for (i = 0; i < leng - 1; i++) {
    m[i] = m9[8];  //output 1bit data
    for (j = 9; j > 1; j--) {
      m9[j - 1] = m9[j - 2]; //1bit shift
    }
    m9[0] = m9[5] ^ m[i]; //xor
  }
  m[leng] = 0;  //last bit is 0
}

void zeroDAC() {
  spiSender(1, 0);
  spiSender(2, 0);
  spiSender(3, 0);
  spiSender(4, 0);
}

void middle() {
  long ave1, ave2, ave3, ave4;
  ave1 = 0;
  ave2 = 0;
  ave3 = 0;
  ave4 = 0;
  delay(1000);
  for (i = 0; i < 10; i++) {
    ave4 = ave4 + analogRead(A7);
    ave1 = ave1 + analogRead(A4);
    ave2 = ave2 + analogRead(A5);
    ave3 = ave3 + analogRead(A6);

  }
  midRec2[0] = ave1 / 10;
  midRec2[1] = ave2 / 10;
  midRec2[2] = ave3 / 10;
  midRec2[3] = ave4 / 10;
}

void middle2() {
  midRec2[0] = analogRead(A4);
  midRec2[1] = analogRead(A5);
  midRec2[2] = analogRead(A6);
  midRec2[3] = analogRead(A7);
}

void middle3() {
  midRec2[0] = (1000 + 250) / 2;
  midRec2[1] = (1000 + 250) / 2;
  midRec2[2] = (1000 + 250) / 2;
  midRec2[3] = (1000 + 250) / 2;
}

void middle4() {
  midRec2[0] = (upperlevel[0] + lowerlevel[0]) /2;
  midRec2[1] = (upperlevel[1] + lowerlevel[1]) /2;
  midRec2[2] = (upperlevel[2] + lowerlevel[2]) /2;
  midRec2[3] = (upperlevel[3] + lowerlevel[3]) /2;
}

void onezero() {
  for (i = 0; i < 511; i += 4) {
    if (m[i - 1] == 0) {
      m[i] = 1;
      m[i + 1] = 1;
      m[i + 2] = 1;
      m[i + 3] = 1;
    }
    else {
      m[i] = 0;
      m[i + 1] = 0;
      m[i + 2] = 0;
      m[i + 3] = 0;
    }
  }
}

void JudgeData(int i) {
  ///////////  1  ////////////
  if (acRec[0] >= midRec2[0]) {
    if (m[i] == 0) bat[0] += 1;
  }
  else {
    if (m[i] == 1) bat[0] += 1;
  }
  ///////////  2  ////////////
  if (acRec[1] >= midRec2[1]) {
    if (m2[i] == 0) bat[1] += 1;
  }
  else {
    if (m2[i] == 1) bat[1] += 1;
  }
  ///////////  3  ////////////
  if (acRec[2] >= midRec2[2]) {
    if (m3[i] == 0) bat[2] += 1;
  }
  else {
    if (m3[i] == 1) bat[2] += 1;
  }
  ///////////  4  ////////////
  if (acRec[3] >= midRec2[3]) {
    if (m4[i] == 0) bat[3] += 1;
  }
  else {
    if (m4[i] == 1) bat[3] += 1;
  }
}

void checkArea(){
  //needs two integer
  //upperlevel and lowerlevel
  int zeroA4;
  int zeroA5;
  int zeroA6;
  int zeroA7;

  for(i=0; i<4; i++){
    upperlevel[i] = 0;
    lowerlevel[i] = 0;
  }

  zeroDAC();
  delay(2);
  zeroA4 = analogRead(A4);
  zeroA5 = analogRead(A5);
  zeroA6 = analogRead(A6);
  zeroA7 = analogRead(A7);

  int acA4[5] = {zeroA4, zeroA4, zeroA4, zeroA4, zeroA4};
  int acA5[5] = {zeroA5, zeroA5, zeroA5, zeroA5, zeroA5};
  int acA6[5] = {zeroA6, zeroA6, zeroA6, zeroA6, zeroA6};
  int acA7[5] = {zeroA7, zeroA7, zeroA7, zeroA7, zeroA7};

  for(i=0; i<2040; i+=10){
    zeroDAC();
    delay(2);
    spiSender(1, i);
    spiSender(2, i);
    spiSender(3, i);
    spiSender(4, i);
    delay(2);
    acA4[0] = analogRead(A4);
    acA5[0] = analogRead(A5);
    acA6[0] = analogRead(A6);
    acA7[0] = analogRead(A7);

    if((acA4[2]-zeroA4)>=4 && (acA4[0]-zeroA4)>=4 && i > 40 && lowerlevel[0] == 0){
      lowerlevel[0] = i;
    }
    if((acA5[2]-zeroA5)>=4 && (acA5[0]-zeroA5)>=4 && i > 40 && lowerlevel[1] == 0){
      lowerlevel[1] = i;
    }
    if((acA6[2]-zeroA6)>=4 && (acA6[0]-zeroA6)>=4 && i > 40 && lowerlevel[2] == 0){
      lowerlevel[2] = i;
    }
    if((acA7[2]-zeroA7)>=4 && (acA7[0]-zeroA7)>=4 && i > 40 && lowerlevel[3] == 0){
      lowerlevel[3] = i;
    }

    if(acA4[0] >= 1000 && upperlevel[0] == 0){
      upperlevel[0] = i;
    }
    if(acA5[0] >= 1000 && upperlevel[1] == 0){
      upperlevel[1] = i;
    }
    if(acA6[0] >= 1000 && upperlevel[2] == 0){
      upperlevel[2] = i;
    }
    if(acA7[0] >= 1000 && upperlevel[3] == 0){
      upperlevel[3] = i;
    }

    for(j=5; j>1; j--){
      acA4[j-1] = acA4[j-2];
      acA5[j-1] = acA5[j-2];
      acA6[j-1] = acA6[j-2];
      acA7[j-1] = acA7[j-2];
    }
  }
}

void mseqShift(int i){
  //////////  2  /////////
  for(j = 0; j<(512-i); j++){
    m2[j] = m[j+i];
  }
  for(j=512-i; j < 512; j++){
    m2[j] = m[j+i-512];
  }
  //////////  3  /////////
  for(j = 0; j<(512-i); j++){
    m3[j] = m2[j+i];
  }
  for(j=512-i; j < 512; j++){
    m3[j] = m2[j+i-512];
  }
  //////////  4  /////////
  for(j = 0; j<(512-i); j++){
    m4[j] = m3[j+i];
  }
  for(j=512-i; j < 512; j++){
    m4[j] = m3[j+i-512];
  }
}
