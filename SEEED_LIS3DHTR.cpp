/*
The value is expressed as two's complement left-justified
Left justification means: all value bits are shifted to the left. How many bits? That depends on your precision.
source: https://stackoverflow.com/questions/49467573/convert-twos-complement-left-justified-integer-into-regular-binary

Example for 10-bit output
╔═════════╦═══════════╦═══════════╗
║         ║  OUT_X_H  ║  OUT_X_L  ║
╠═════════╬═══════════╬═══════════╣
║ Content ║ xxxx xxxx ║ xx00 0000 ║
╚═════════╩═══════════╩═══════════╝

Data for each axis is split into two 8-bit registers.
We need to store this as an int16_t, then left shift by (16 - precision) (i.e. 16 - 10 = 6 for 10-bit output)
This value is multiplied by the sensitivity to get the result in gs.
*/

#include "Arduino.h"
#include "SEEED_LIS3DHTR.h"

//
LIS3DHTR_I2C::LIS3DHTR_I2C() {}



/* Defaults: ax,ay,az enabled 1Hz ODR 2G Range */
void LIS3DHTR_I2C::init(TwoWire &comm, uint8_t dev_addr)
{
    _wire_com = &comm;
    _wire_com->begin();
    i2c_addr = dev_addr;
    setODR(ODR_1Hz);
    delay(100);
    current_RANGE = RANGE_2G;  // default
    current_RESO = RESO_NORMAL;// default
    sensitivity = 0.004;    // default
    delay(100);
}

void LIS3DHTR_I2C::setFIFO(fifo_t fifo)
{
    uint8_t data = readRegister(CTRL_REG5);
    data |= CTRL_REG5_FIFO_EN;
    writeRegister(CTRL_REG5, data);
    delay(100);
    data = readRegister(FIFO_CTRL_REG);
    data &= 0b00111111;
    data |= (fifo << 7);
    writeRegister(FIFO_CTRL_REG, data);
    delay(100);
}

uint8_t LIS3DHTR_I2C::getFIFOSamples()
{
    return readRegister(FIFO_SRC_REG) & 0b00011111;
}


bool LIS3DHTR_I2C::available()
{
    return readRegister(STATUS_REG) & STATUS_REG_ZYXDA;
}

void LIS3DHTR_I2C::writeRegister(uint8_t reg, uint8_t val)
{
    _wire_com->beginTransmission(i2c_addr);
    _wire_com->write(reg);
    _wire_com->write(val);
    _wire_com->endTransmission();
}

void LIS3DHTR_I2C::readRegisterRegion(uint8_t *data, uint8_t reg, uint8_t len)
{
    _wire_com->beginTransmission(i2c_addr);
    reg |= 0x80;  //turn auto-increment bit on, bit 7 for I2C, discussed in 6.1.1 of manual
    _wire_com->write(reg);
    _wire_com->endTransmission();
    _wire_com->requestFrom(i2c_addr, len);
    int i = 0;
    while ((_wire_com->available()) && (i++ < len)) // slave may send less than requested
    {
        *data++ = _wire_com->read(); // receive a byte
    }
}

uint8_t LIS3DHTR_I2C::readRegister(uint8_t reg)
{
    uint8_t data;
    readRegisterRegion(&data, reg, 1);
    return data;
}


void LIS3DHTR_I2C::setResolution(reso_t reso)
{
    //low-power 8-bit, normal 10-bit, high reso 12-bit
    uint8_t data = readRegister(CTRL_REG1);
    if (RESO_LOW)
    {
        data |= CTRL_REG1_LPen; // set LPen bit 
    }
    else
    {
        data &= ~CTRL_REG1_LPen; // clear LPen bit
    }
    writeRegister(CTRL_REG1, data);
    
    data = readRegister(CTRL_REG4);
    if (reso == RESO_HIGH)
    {
        data |= CTRL_REG4_HR; // set bit
    }
    else
    {
        data &= ~CTRL_REG4_HR;  // clear HR bit
    }
    writeRegister(CTRL_REG4, data);
    
    current_RESO = reso;
    int turn_on_time = 0;
    if (reso == RESO_HIGH)
    {
        turn_on_time = 7.0/current_ODR;
    }
    else
    {
        turn_on_time = 1.0/current_ODR;
    }
    delay(turn_on_time);
    
    updateSensitivity();

    switch(current_RESO)
    {
        case RESO_HIGH:
            unused_bits = 4;
            break;
        case RESO_NORMAL:
            unused_bits = 6;
            break;
        case RESO_LOW:
            unused_bits = 8;
            break;
    }
    Serial.print("Unused bits = ");
    Serial.println(unused_bits);
    delay(100);
}


void LIS3DHTR_I2C::updateSensitivity()
{
    float So;
    switch(current_RANGE)
    {
        case RANGE_2G:
            So = 1;
            break;
        case RANGE_4G:
            So = 2;
            break;
        case RANGE_8G:
            So = 4;
            break;
        case RANGE_16G:
            So = 12;
            break;
    }
    switch(current_RESO)
    {
        case RESO_HIGH:
            break;  // *= 1
        case RESO_NORMAL:
            So *= 4;
            break;
        case RESO_LOW:
            So *= 16;
            break;
    }
    sensitivity = (So / 1000.0);
    Serial.print("Sensitivity = ");
    Serial.println(sensitivity, 4);
}

void LIS3DHTR_I2C::getAcceleration(float *x, float *y, float *z)
{
    // Read the Accelerometer
    uint8_t buf[6] = {0};
    readRegisterRegion(buf, OUT_X_L,6);

    

 //   Serial.println(mask);
    int16_t ax = ((int16_t*)buf)[0] >> unused_bits;
    int16_t ay = ((int16_t*)buf)[1] >> unused_bits;
    int16_t az = ((int16_t*)buf)[2] >> unused_bits;
    
    float fax = ax * sensitivity; // 0.004 is sensitivity for +-2G with normal output, will need to change for others
    float fay = ay * sensitivity;
    float faz = az * sensitivity;

    *x = fax;       // 16-bit signed result for X-Axis Acceleration Data of LIS3DHTR_I2C
    *y = fay;       // 16-bit signed result for Y-Axis Acceleration Data of LIS3DHTR_I2C
    *z = faz;       // 16-bit signed result for Z-Axis Acceleration Data of LIS3DHTR_I2C
}

float LIS3DHTR_I2C::getSensitivity()
{
    return sensitivity;
}


void LIS3DHTR_I2C::setODR(odr_t odr)
{
    uint8_t data = readRegister(CTRL_REG1);
    data &= 0x0F;
    data |= ((uint8_t)odr << ODR_OFFSET);
    writeRegister(CTRL_REG1, data);
    current_ODR = odr;
    delay(100);
}

void LIS3DHTR_I2C::setRange(range_t range)
{
    uint8_t data = readRegister(CTRL_REG4);
    data &= 0b11001111;
    data |= ((uint8_t)range << RANGE_OFFSET);
    writeRegister(CTRL_REG4, data);
    current_RANGE = range;
    updateSensitivity();
    delay(100);
}