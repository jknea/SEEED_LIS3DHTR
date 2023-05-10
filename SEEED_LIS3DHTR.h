#ifndef SEEED_LIS3DHTR_h
#define SEEED_LIS3DHTR_h

#include "Arduino.h"

// Reserved 0x0-0x06
#define STATUS_REG_AUX      0x07
#define OUT_ADC1_L          0x08
#define OUT_ADC1_H          0x09
#define OUT_ADC2_L          0x0A
#define OUT_ADC2_H          0x0B
#define OUT_ADC3_L          0x0C
#define OUT_ADC3_H          0x0D
// Reserved 0x0E
#define WHO_AM_I            0x0F
// Reserved 0x10-0x1D
#define CTRL_REG0           0x1E
#define TEMP_CFG_REG        0x1F
#define CTRL_REG1           0x20
#define CTRL_REG2           0x21
#define CTRL_REG3           0x22
#define CTRL_REG4           0x23
#define CTRL_REG5           0x24
#define CTRL_REG6           0x25
#define REFERENCE           0x26
#define STATUS_REG          0x27
#define OUT_X_L             0x28
#define OUT_X_H             0x29
#define OUT_Y_L             0x2A
#define OUT_Y_H             0x2B
#define OUT_Z_L             0x2C
#define OUT_Z_H             0x2D
#define FIFO_CTRL_REG       0x2E
#define FIFO_SRC_REG        0x2F
#define INT1_CFG            0x30
#define INT1_SRC            0x31
#define INT1_THS            0x32
#define INT1_DURATION       0x33
#define INT2_CFG            0x34
#define INT2_SRC            0x35
#define INT2_THS            0x36
#define INT2_DURATION       0x37
#define CLICK_CFG           0x38
#define CLICK_SRC           0x39
#define CLICK_THS           0x3A
#define TIME_LIMIT          0x3B
#define TIME_LATENCY        0x3C
#define TIME_WINDOW         0x3D
#define ACT_THS             0x3E
#define ACT_DUR             0x3F

/* STATUS_REG_AUX field masks */
#define STATUS_REG_321OR               ((unsigned char)1 << 7) // 1-axis new data available
#define STATUS_REG_3OR                 ((unsigned char)1 << 6) // 2-axis new data available
#define STATUS_REG_2OR                 ((unsigned char)1 << 5) // 3-axis new data available 
#define STATUS_REG_1OR                 ((unsigned char)1 << 4) // 1, 2 and 3-axis new data available
#define STATUS_REG_321DA               ((unsigned char)1 << 3) // 1-axis data overrun
#define STATUS_REG_3DA                 ((unsigned char)1 << 2) // 2-axis data overrun
#define STATUS_REG_2DA                 ((unsigned char)1 << 1) // 3-axis data overrun
#define STATUS_REG_1DA                 ((unsigned char)1 << 0) // 1, 2 and 3-axis data overrun

/* CTRL_REG0 field masks */
#define CTRL_REG0_SDO_PU_DISC          ((unsigned char)1 << 7) // Disconnect SDO/SA0 pull-up

/* TEMP_CFG_REG field masks */
#define TEMP_CFG_REG_ADC_EN            ((unsigned char)1 << 7) // ADC enable
#define TEMP_CFG_REG_TEMP_EN           ((unsigned char)1 << 6) // Temperature sensor (T) enable


/* CTRL_REG1 field masks */
#define CTRL_REG1_ODR3                 ((unsigned char)1 << 7)
#define CTRL_REG1_ODR2                 ((unsigned char)1 << 6)
#define CTRL_REG1_ODR1                 ((unsigned char)1 << 5)
#define CTRL_REG1_ODR0                 ((unsigned char)1 << 4)
#define CTRL_REG1_LPen                 ((unsigned char)1 << 3)
#define CTRL_REG1_Zen                  ((unsigned char)1 << 2)
#define CTRL_REG1_Yen                  ((unsigned char)1 << 1)
#define CTRL_REG1_Xen                  ((unsigned char)1 << 0)

/* CTRL_REG2 field masks */
#define CTRL_REG2_HPM1                 ((unsigned char)1 << 7)        
#define CTRL_REG2_HPM0                 ((unsigned char)1 << 6)                     
#define CTRL_REG2_HPCF2                ((unsigned char)1 << 5)                     
#define CTRL_REG2_HPCF1                ((unsigned char)1 << 4)                     
#define CTRL_REG2_FDS                  ((unsigned char)1 << 3)                     
#define CTRL_REG2_HPCLICK              ((unsigned char)1 << 2)                         
#define CTRL_REG2_HP_IA2               ((unsigned char)1 << 1)                     
#define CTRL_REG2_HP_IA1               ((unsigned char)1 << 0)                     

/* CTRL_REG3 field masks */
#define CTRL_REG3_I1_CLICK             ((unsigned char)1 << 7)
#define CTRL_REG3_I1_IA1               ((unsigned char)1 << 6)
#define CTRL_REG3_I1_IA2               ((unsigned char)1 << 5)
#define CTRL_REG3_I1_ZYXDA             ((unsigned char)1 << 4)
#define CTRL_REG3_I1_321DA             ((unsigned char)1 << 3)
#define CTRL_REG3_I1_WTM               ((unsigned char)1 << 2)
#define CTRL_REG3_I1_OVERRUN           ((unsigned char)1 << 1)

/* CTRL_REG4 field masks */
#define CTRL_REG4_BDU                  ((unsigned char)1 << 7) 
#define CTRL_REG4_BLE                  ((unsigned char)1 << 6)
#define CTRL_REG4_FS1                  ((unsigned char)1 << 5) 
#define CTRL_REG4_FS0                  ((unsigned char)1 << 4) 
#define CTRL_REG4_HR                   ((unsigned char)1 << 3)
#define CTRL_REG4_ST1                  ((unsigned char)1 << 2) 
#define CTRL_REG4_ST0                  ((unsigned char)1 << 1) 
#define CTRL_REG4_SIM                  ((unsigned char)1 << 0)

/* CTRL_REG5 field masks */
#define CTRL_REG5_BOOT                  ((unsigned char)1 << 7) 
#define CTRL_REG5_FIFO_EN                  ((unsigned char)1 << 6)
//#define CTRL_REG5_FS1                  ((unsigned char)1 << 5) 
//#define CTRL_REG5_FS0                  ((unsigned char)1 << 4) 
#define CTRL_REG5_LIR_INT1                   ((unsigned char)1 << 3)
#define CTRL_REG5_D4D_INT1                  ((unsigned char)1 << 2) 
#define CTRL_REG5_LIR_INT2                  ((unsigned char)1 << 1) 
#define CTRL_REG5_D4D_INT2                  ((unsigned char)1 << 0)

/* CTRL_REG6 field masks */
#define CTRL_REG6_I2_CLICK                  ((unsigned char)1 << 7) 
#define CTRL_REG6_I2_IA1                  ((unsigned char)1 << 6)
#define CTRL_REG6_I2_IA2                  ((unsigned char)1 << 5) 
#define CTRL_REG6_I2_BOOT                  ((unsigned char)1 << 4) 
#define CTRL_REG4_I2_ACT                   ((unsigned char)1 << 3)
//#define CTRL_REG4_ST1                  ((unsigned char)1 << 2) 
#define CTRL_REG6_INT_POLARITY                  ((unsigned char)1 << 1) 
//#define CTRL_REG4_SIM                  ((unsigned char)1 << 0)


/* STATUS_REG field masks */
#define STATUS_REG_ZYXOR                ((unsigned char)1 << 7)
#define STATUS_REG_ZOR              ((unsigned char)1 << 6)
#define STATUS_REG_YOR              ((unsigned char)1 << 5)
#define STATUS_REG_XOR              ((unsigned char)1 << 4)
#define STATUS_REG_ZYXDA                ((unsigned char)1 << 3)
#define STATUS_REG_ZDA              ((unsigned char)1 << 2)
#define STATUS_REG_YDA              ((unsigned char)1 << 1)
#define STATUS_REG_XDA              ((unsigned char)1 << 0)

/* FIFO_CTRL_REG field masks */
#define FIFO_CTRL_REG_FM1              ((unsigned char)1 << 7)  
#define FIFO_CTRL_REG_FM0              ((unsigned char)1 << 6)  
#define FIFO_CTRL_REG_TR               ((unsigned char)1 << 5)  
#define FIFO_CTRL_REG_FTH4             ((unsigned char)1 << 4)      
#define FIFO_CTRL_REG_FTH3             ((unsigned char)1 << 3)      
#define FIFO_CTRL_REG_FTH2             ((unsigned char)1 << 2)      
#define FIFO_CTRL_REG_FTH1             ((unsigned char)1 << 1)      
#define FIFO_CTRL_REG_FTH0             ((unsigned char)1 << 0)

/* FIFO_SRC_REG field masks */
#define FIFO_SRC_REG_WTM               ((unsigned char)1 << 7)      
#define FIFO_SRC_REG_OVRN_FIFO         ((unsigned char)1 << 6)          
#define FIFO_SRC_REG_EMPTY             ((unsigned char)1 << 5)      
#define FIFO_SRC_REG_FSS4              ((unsigned char)1 << 4)      
#define FIFO_SRC_REG_FSS3              ((unsigned char)1 << 3)      
#define FIFO_SRC_REG_FSS2              ((unsigned char)1 << 2)      
#define FIFO_SRC_REG_FSS1              ((unsigned char)1 << 1)      
#define FIFO_SRC_REG_FSS0              ((unsigned char)1 << 0)      

/* INT1_CFG field masks */
#define INT1_CFG_AOI                   ((unsigned char)1 << 7)           
#define INT1_CFG_6D                    ((unsigned char)1 << 6)       
#define INT1_CFG_ZHIE                  ((unsigned char)1 << 5)           
#define INT1_CFG_ZLIE                  ((unsigned char)1 << 4)           
#define INT1_CFG_YHIE                  ((unsigned char)1 << 3)           
#define INT1_CFG_YLIE                  ((unsigned char)1 << 2)           
#define INT1_CFG_XHIE                  ((unsigned char)1 << 1)           
#define INT1_CFG_XLIE                  ((unsigned char)1 << 0)           

/* INT1_SRC field masks */
//#define INT1_SRC_0                      ((unsigned char)1 << 7)     
#define INT1_SRC_IA                     ((unsigned char)1 << 6)     
#define INT1_SRC_ZH                     ((unsigned char)1 << 5)     
#define INT1_SRC_ZL                     ((unsigned char)1 << 4)     
#define INT1_SRC_YH                     ((unsigned char)1 << 3)     
#define INT1_SRC_YL                     ((unsigned char)1 << 2)     
#define INT1_SRC_XH                     ((unsigned char)1 << 1)     
#define INT1_SRC_XL                     ((unsigned char)1 << 0)     




//#define INT1_THS_0          ((unsigned char)1 << 7)       
#define INT1_THS_THS6       ((unsigned char)1 << 6)           
#define INT1_THS_THS5       ((unsigned char)1 << 5)           
#define INT1_THS_THS4       ((unsigned char)1 << 4)           
#define INT1_THS_THS3       ((unsigned char)1 << 3)           
#define INT1_THS_THS2       ((unsigned char)1 << 2)           
#define INT1_THS_THS1       ((unsigned char)1 << 1)           
#define INT1_THS_THS0       ((unsigned char)1 << 0)           


//#define INT1_DURATION_0     ((unsigned char)1 << 7)     
#define INT1_DURATION_D6    ((unsigned char)1 << 6)     
#define INT1_DURATION_D5    ((unsigned char)1 << 5)     
#define INT1_DURATION_D4    ((unsigned char)1 << 4)     
#define INT1_DURATION_D3    ((unsigned char)1 << 3)     
#define INT1_DURATION_D2    ((unsigned char)1 << 2)     
#define INT1_DURATION_D1    ((unsigned char)1 << 1)     
#define INT1_DURATION_D0    ((unsigned char)1 << 0)     





/* INT2_CFG field masks */
#define INT2_CFG_AOI                   ((unsigned char)1 << 7)           
#define INT2_CFG_6D                    ((unsigned char)1 << 6)       
#define INT2_CFG_ZHIE                  ((unsigned char)1 << 5)           
#define INT2_CFG_ZLIE                  ((unsigned char)1 << 4)           
#define INT2_CFG_YHIE                  ((unsigned char)1 << 3)           
#define INT2_CFG_YLIE                  ((unsigned char)1 << 2)           
#define INT2_CFG_XHIE                  ((unsigned char)1 << 1)           
#define INT2_CFG_XLIE                  ((unsigned char)1 << 0)           

/* INT2_SRC field masks */
//#define INT1_SRC_0                      ((unsigned char)1 << 7)     
#define INT2_SRC_IA                     ((unsigned char)1 << 6)     
#define INT2_SRC_ZH                     ((unsigned char)1 << 5)     
#define INT2_SRC_ZL                     ((unsigned char)1 << 4)     
#define INT2_SRC_YH                     ((unsigned char)1 << 3)     
#define INT2_SRC_YL                     ((unsigned char)1 << 2)     
#define INT2_SRC_XH                     ((unsigned char)1 << 1)     
#define INT2_SRC_XL                     ((unsigned char)1 << 0)     




//#define INT2_THS_0          ((unsigned char)1 << 7)       
#define INT2_THS_THS6       ((unsigned char)1 << 6)           
#define INT2_THS_THS5       ((unsigned char)1 << 5)           
#define INT2_THS_THS4       ((unsigned char)1 << 4)           
#define INT2_THS_THS3       ((unsigned char)1 << 3)           
#define INT2_THS_THS2       ((unsigned char)1 << 2)           
#define INT2_THS_THS1       ((unsigned char)1 << 1)           
#define INT2_THS_THS0       ((unsigned char)1 << 0)           


//#define INT1_DURATION_0     ((unsigned char)1 << 7)     
#define INT2_DURATION_D6    ((unsigned char)1 << 6)     
#define INT2_DURATION_D5    ((unsigned char)1 << 5)     
#define INT2_DURATION_D4    ((unsigned char)1 << 4)     
#define INT2_DURATION_D3    ((unsigned char)1 << 3)     
#define INT2_DURATION_D2    ((unsigned char)1 << 2)     
#define INT2_DURATION_D1    ((unsigned char)1 << 1)     
#define INT2_DURATION_D0    ((unsigned char)1 << 0)     


//#define CLICK_CFG_--    ((unsigned char)1 << 7)      
//#define CLICK_CFG_--    ((unsigned char)1 << 6)      
#define CLICK_CFG_zd    ((unsigned char)1 << 5)      
#define CLICK_CFG_zs    ((unsigned char)1 << 4)      
#define CLICK_CFG_yd    ((unsigned char)1 << 3)      
#define CLICK_CFG_ys    ((unsigned char)1 << 2)      
#define CLICK_CFG_xd    ((unsigned char)1 << 1)      
#define CLICK_CFG_xs    ((unsigned char)1 << 0)      

//#define CLICK_SRC_0      ((unsigned char)1 << 7)         
#define CLICK_SRC_IA     ((unsigned char)1 << 6)         
#define CLICK_SRC_DCLICK ((unsigned char)1 << 5)             
#define CLICK_SRC_SCLICK ((unsigned char)1 << 4)             
#define CLICK_SRC_Sign   ((unsigned char)1 << 3)             
#define CLICK_SRC_Z      ((unsigned char)1 << 2)         
#define CLICK_SRC_Y      ((unsigned char)1 << 1)         
#define CLICK_SRC_X      ((unsigned char)1 << 0)         



#define CLICK_THS_LIR_Click     ((unsigned char)1 << 7)               
#define CLICK_THS_Ths6          ((unsigned char)1 << 6)           
#define CLICK_THS_Ths5          ((unsigned char)1 << 5)           
#define CLICK_THS_Ths4          ((unsigned char)1 << 4)           
#define CLICK_THS_Ths3          ((unsigned char)1 << 3)           
#define CLICK_THS_Ths2          ((unsigned char)1 << 2)           
#define CLICK_THS_Ths1          ((unsigned char)1 << 1)           
#define CLICK_THS_Ths0          ((unsigned char)1 << 0)           


//#define TIME_LIMIT_-        ((unsigned char)1 << 7)           
#define TIME_LIMIT_TLI6     ((unsigned char)1 << 6)               
#define TIME_LIMIT_TLI5     ((unsigned char)1 << 5)               
#define TIME_LIMIT_TLI4     ((unsigned char)1 << 4)               
#define TIME_LIMIT_TLI3     ((unsigned char)1 << 3)               
#define TIME_LIMIT_TLI2     ((unsigned char)1 << 2)               
#define TIME_LIMIT_TLI1     ((unsigned char)1 << 1)               
#define TIME_LIMIT_TLI0     ((unsigned char)1 << 0)               




#define I2C_DEFAULT_ADDR 0x19


#define ODR_OFFSET 4
#define RANGE_OFFSET 4

#include <Arduino.h>
#include <Wire.h>

// output data rate
enum odr_t
{
    POWER_DOWN,
    ODR_1Hz,
    ODR_10Hz,
    ODR_25Hz,
    ODR_50Hz,
    ODR_100Hz,
    ODR_200Hz,
    ODR_400Hz,
    ODR_1600Hz,
    ODR_5300Hz
};

// measurement rage
enum range_t 
{
    RANGE_2G,
    RANGE_4G,
    RANGE_8G,
    RANGE_16G
};

enum fifo_t
{
    BYPASS,
    FIFO,
    STREAM,
    STREAM_TO_FIFO
};

enum reso_t
{
    RESO_LOW,
    RESO_NORMAL,
    RESO_HIGH
};

class LIS3DHTR_I2C
{
    public:
        LIS3DHTR_I2C();
        void init(TwoWire &comm = Wire, uint8_t dev_addr = I2C_DEFAULT_ADDR);
        void setODR(odr_t odr);
        void setFIFO(fifo_t fifo);
        void setRange(range_t range);
        void setResolution(reso_t reso);
        float getSensitivity();
        uint8_t getFIFOSamples();
        void getAcceleration(float *x, float *y, float *z);
        bool available();
        //operator bool();
    private:
        void updateSensitivity();
        void writeRegister(uint8_t reg, uint8_t val);
        uint8_t readRegister(uint8_t reg);
        void readRegisterRegion(uint8_t *data, uint8_t reg, uint8_t n);
        uint8_t i2c_addr;
        TwoWire *_wire_com;
        odr_t current_ODR;
        range_t current_RANGE;
        reso_t current_RESO;
        float sensitivity;
        unsigned unused_bits;
};


#endif SEEED_LIS3DHTR_h