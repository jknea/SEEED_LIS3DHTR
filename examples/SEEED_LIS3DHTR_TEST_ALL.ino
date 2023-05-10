#include "SEEED_LIS3DHTR.h"

LIS3DHTR_I2C LIS;

float ax, ay, az;

float max_x = 0, max_y = 0, max_z = 0;
float min_x = 0, min_y = 0, min_z = 0;

unsigned long t0;


void printAccels();
void testReso();
void testODR();

void setup() {
  Serial.begin(115200);
  delay(5000);
  Serial.println("Serial success!!!");
  LIS.init(Wire, 0x19);
  delay(2000);
  LIS.setODR(ODR_1Hz);
  LIS.setRange(RANGE_2G);
  t0 = micros();
}

int count = 0;

void loop() {
  testReso();
  testODR();
}


void testODR()
{
  LIS.setResolution(RESO_LOW);
  odr_t odrs[7] = {ODR_1Hz, ODR_10Hz, ODR_25Hz, ODR_50Hz, ODR_100Hz, ODR_200Hz, ODR_400Hz}; //, ODR_1600Hz, ODR_5300Hz};
  for (int i = 0; i < 7; i++)
  {
    LIS.setODR(odrs[i]);
    unsigned long t0 = micros();
    unsigned count = 0;
    while (micros() - t0 < 1000000)
    {
      if (LIS.available())
      {
        count++;
        LIS.getAcceleration(&ax, &ay, &az);
        //printAccels();
      }
    }
    Serial.print("Samples read in 1s = ");
    Serial.println(count);
    delay(1000);
  }
}

void testRange()
{
  Serial.println("Setting RANGE_2G");
  LIS.setRange(RANGE_2G);
  while (count < 10)
  {
    if (LIS.available()) 
    {
      LIS.getAcceleration(&ax, &ay, &az);
      printAccels();
      count++;
    }
  }
  count = 0;
}


void testReso()
{
  Serial.println("Setting RESO_LOW");
  LIS.setResolution(RESO_LOW);
  while (count < 10)
  {
    if (LIS.available()) 
    {
      LIS.getAcceleration(&ax, &ay, &az);
      printAccels();
      count++;
    }
  }
  count = 0;
  Serial.println();

  Serial.println("Setting RESO_NORMAL");
  LIS.setResolution(RESO_NORMAL);
  while (count < 10)
  {
    if (LIS.available()) 
    {
      LIS.getAcceleration(&ax, &ay, &az);
      printAccels();
      count++;
    }
  }
  count = 0;
  Serial.println();

  Serial.println("Setting RESO_HIGH");
  LIS.setResolution(RESO_HIGH);
  while (count < 10)
  {
    if (LIS.available()) 
    {
      LIS.getAcceleration(&ax, &ay, &az);
      printAccels();
      count++;
    }
  }
  count = 0;
  Serial.println();

}

void printAccels()
{
  Serial.print(ax, 4);
  Serial.print(", ");
  Serial.print(ay, 4);
  Serial.print(", ");
  Serial.println(az, 4);
}