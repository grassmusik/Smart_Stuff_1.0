/*======================================================================================*/
/*              Michael Martin - Simple Biomedical Project                              */
/*                                                                                      */
/*                                                                                      */
/*======================================================================================*/

#include <mbed.h>                                                                       //Include library
#include "MPU9250.h"

    Serial pc1(USBTX, USBRX);                                                           //Declare serial port printing with variable 'pc1'
    mpu9250_spi::mpu9250_spi(SPI& _spi, PinName _cs) : spi(_spi), cs(_cs) {}
    
/****************************
*                                   ACCEL_GYRO_REGISTER WRITE
*   call this function to 
*   returns Factory Trim value
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/  
unsigned int mpu9250_spi::WriteReg( uint8_t WriteAddr, uint8_t WriteData )              //Write Register Function
    {
    unsigned int temp_val;
    select();
    spi.write(WriteAddr);
    temp_val=spi.write(WriteData);
    deselect();
    wait(.1);
    return temp_val;
    }

/****************************
*                                   ACCEL_GYRO_REGISTER READ
*   call this function to 
*   returns Factory Trim value
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/     
unsigned int  mpu9250_spi::ReadReg( uint8_t WriteAddr, uint8_t WriteData )              //Read Register Function
    {
    return WriteReg(WriteAddr | READ_FLAG,WriteData);
    }

/****************************
*                                  ACCEL_GYRO_REGISTERS READ
*   call this function to 
*   returns Factory Trim value
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/      
void mpu9250_spi::ReadRegs( uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes )    //Read Registers Function
    {
    unsigned int  i = 0;

    select();
    spi.write(ReadAddr | READ_FLAG);
    for(i=0; i<Bytes; i++)
        ReadBuf[i] = spi.write(0x00);
    deselect();
    wait(.1);
    }

/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
*   call this function at startup, giving the sample rate divider (raging from 0 to 255) and
*   low pass filter value; suitable values are:
*   returns 1 if an error occurred
*   BITS_DLPF_CFG_256HZ_NOLPF2
*   BITS_DLPF_CFG_188HZ
*   BITS_DLPF_CFG_98HZ
*   BITS_DLPF_CFG_42HZ
*   BITS_DLPF_CFG_20HZ
*   BITS_DLPF_CFG_10HZ 
*   BITS_DLPF_CFG_5HZ 
*   BITS_DLPF_CFG_2100HZ_NOLPF
*
-----------------------------------------------------------------------------------------------*/
#define MPU_InitRegNum 17

/****************************
*                                   ACCEL_GYRO_MAG INITIALIZATION
*   call this function to 
*   returns Factory Trim value
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/ 
bool mpu9250_spi::init(int sample_rate_div,int low_pass_filter)                         //Initiliazation Function
    {
    uint8_t i = 0;
    select();
    uint8_t MPU_Init_Data[MPU_InitRegNum][2] = 
        {
        {0x80, MPUREG_PWR_MGMT_1},                                                      // Reset Device
        {0x01, MPUREG_PWR_MGMT_1},                                                      // Clock Source
        {0x00, MPUREG_PWR_MGMT_2},                                                      // Enable Acc & Gyro
        {low_pass_filter, MPUREG_CONFIG},                                               // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {0x18, MPUREG_GYRO_CONFIG},                                                     // +-2000dps
        {0x08, MPUREG_ACCEL_CONFIG},                                                    // +-4G
        {0x09, MPUREG_ACCEL_CONFIG_2},                                                  // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x22, MPUREG_INT_PIN_CFG},                                                     //
        //{0x40, MPUREG_I2C_MST_CTRL},                                                  // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},                                                     // Enable AUX
        {0x30, MPUREG_USER_CTRL},                                                       // I2C Master mode
        {0x0D, MPUREG_I2C_MST_CTRL},                                                    //I2C configuration multi-master  IIC 400KHz
        
        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},                                        //Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL},                                              //Enable I2C delay

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG},                                            //I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO},                                                     // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL},                                                   //Enable I2C and set 1 byte

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG},                                            //I2C slave 0 register address from where to begin data transfer
        {0x12, MPUREG_I2C_SLV0_DO},                                                     //Register value to continuous measurement in 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL}                                                    //Enable I2C and set 1 byte
        
        };
        
    spi.format(8,3);
    spi.frequency(1000000);

    for(i=0; i<MPU_InitRegNum; i++) 
        {
        WriteReg(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        wait(0.5);                                                                      //I2C must slow down the write speed, otherwise it won't work
        }
//    WriteReg(MPUREG_USER_CTRL[4], I2C_IF_DIS=1);
    set_acc_scale(2);
    set_gyro_scale(250);
    AK8963_calib_Magnetometer();
    deselect();
    return 0;

    }

/****************************
*                                   ACCEL_SCALE SETTING
*   call this function at startup, after initialization, to set the right range for the accelerometers. Suitable ranges are: 
*   returns the range set (2,4,8 or 16)
*
*   BITS_FS_2G
*   BITS_FS_4G
*   BITS_FS_8G
*   BITS_FS_16G
*
****************************/ 
unsigned int mpu9250_spi::set_acc_scale(int scale)                                      //Set Accelerometer Scale Function
    {
    select();
    unsigned int temp_scale;
    
    WriteReg(MPUREG_ACCEL_CONFIG, scale);
        switch (scale)
            {
            case BITS_FS_2G:
                acc_divider=16384;
            break;
            case BITS_FS_4G:
                acc_divider=8192;
            break;
            case BITS_FS_8G:
                acc_divider=4096;
            break;
            case BITS_FS_16G:
                acc_divider=2048;
            break;  
            }
    temp_scale=WriteReg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);
        switch (temp_scale)
            {
            case BITS_FS_2G:
                temp_scale=2;
            break;
            case BITS_FS_4G:
                temp_scale=4;
            break;
            case BITS_FS_8G:
                temp_scale=8;
            break;
            case BITS_FS_16G:
                temp_scale=16;
            break;   
            }
            
    deselect();
    return temp_scale;
    }

/****************************
*                                   GYRO_SCALE SETTING
*   call this function at startup, after initialization, to set the right range for the gyroscopes.   
*   returns the range set (250,500,1000 or 2000)
*
*   BITS_250DPS
*   BITS_FS_500DPS
*   BITS_FS_1000DPS
*   BITS_FS_2000DPS
*
****************************/  
unsigned int mpu9250_spi::set_gyro_scale(int scale)                                     //Set Gyroscope Scale Function
    {
    select();
    unsigned int temp_scale;
    
    WriteReg(MPUREG_GYRO_CONFIG, scale);
        switch (scale)
            {
            case BITS_FS_250DPS:
                gyro_divider=131;
            break;
            case BITS_FS_500DPS:
                gyro_divider=65.5;
            break;
            case BITS_FS_1000DPS:
                gyro_divider=32.8;
            break;
            case BITS_FS_2000DPS:
                gyro_divider=16.4;
            break;   
            }
    temp_scale=WriteReg(MPUREG_GYRO_CONFIG|READ_FLAG, 0x00);
        switch (temp_scale)
            {
            case BITS_FS_250DPS:
                temp_scale=250;
            break;
            case BITS_FS_500DPS:
                temp_scale=500;
            break;
            case BITS_FS_1000DPS:
                temp_scale=1000;
            break;
            case BITS_FS_2000DPS:
                temp_scale=2000;
            break;   
            }
    deselect();
    return temp_scale;
    }

/****************************
*                                   ACCEL_SCALE SETTING
*   call this function to know if SPI is working correctly. It checks the I2C address of the
*   mpu9250 which should be 104 when in SPI mode.
*   returns the I2C address (104)
*
****************************/
unsigned int mpu9250_spi::whoami()                                                      //IMU WhoamI Register Function
    {
    select();
    unsigned int response;
    response=WriteReg(MPUREG_WHOAMI|READ_FLAG, 0x00);
    deselect();
    return response;
    }

/****************************
*                            ACCEL_Read Data
*   call this function to read accelerometer data. Axis represents selected axis:
*   0 -> X axis
*   1 -> Y axis
*   2 -> Z axis
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/ 
void mpu9250_spi::read_acc()                                                            //Read Acceleration Function
    {
    select();
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPUREG_ACCEL_XOUT_H,response,6);
        for(i=0; i<3; i++)
            {
            bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
            data=(float)bit_data;
            accelerometer_data[i]=data/acc_divider;
            }
    deselect();   
    }

/****************************
*                            GYRO_Read Data
*   call this function to read gyroscope data. Axis represents selected axis:
*   0 -> X axis
*   1 -> Y axis
*   2 -> Z axis
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/
void mpu9250_spi::read_rot()                                                            //Read Gyroscope Function
    {
    select();
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPUREG_GYRO_XOUT_H,response,6);
        for(i=0; i<3; i++) 
            {
            bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
            data=(float)bit_data;
            gyroscope_data[i]=data/gyro_divider;
            }
    deselect();
    }

/****************************
*                            TEMP_Read Data
*   call this function to read temperature data. 
*   returns the value in °C
*   0 -> X axis
*   1 -> Y axis
*   2 -> Z axis
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/
void mpu9250_spi::read_temp()                                                           //Read Temperature Function
    {
    select();
    uint8_t response[2];
    int16_t bit_data;
    float data;
    ReadRegs(MPUREG_TEMP_OUT_H,response,2);

    bit_data=((int16_t)response[0]<<8)|response[1];
    data=(float)bit_data;
    Temperature=(data/340)+36.53;
    deselect();
    }

/****************************
*                            ACCEL_Read CALIBRATION
*   call this function to read accelerometer data. Axis represents selected axis: 
*   returns Factory Trim value
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/
void mpu9250_spi::calib_acc()
    {
    select();
    uint8_t response[4];
    int temp_scale;
    
    temp_scale=WriteReg(MPUREG_ACCEL_CONFIG|READ_FLAG, 0x00);                           //READ CURRENT ACC SCALE
    set_acc_scale(BITS_FS_8G);
                                                                
    //temp_scale=WriteReg(MPUREG_ACCEL_CONFIG, 0x80>>axis);                             //ENABLE SELF TEST need modify

    ReadRegs(MPUREG_SELF_TEST_X,response,4);
    calib_data[0]=((response[0]&11100000)>>3)|((response[3]&00110000)>>4);
    calib_data[1]=((response[1]&11100000)>>3)|((response[3]&00001100)>>2);
    calib_data[2]=((response[2]&11100000)>>3)|((response[3]&00000011));

    set_acc_scale(temp_scale);
    deselect();
    }

/****************************
*                            MAG_WHO AM I Register
*   call this function to 
*   returns Factory Trim value
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/   
uint8_t mpu9250_spi::AK8963_whoami()                                                    //IMU Magnetometer Register Function
    {
    select();
    uint8_t response;
      WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);                           //Set the I2C slave addres of AK8963 and set for read.
      WriteReg(MPUREG_I2C_SLV0_REG, AK8963_WIA);                                          //I2C slave 0 register address from where to begin data transfer
      WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);                                               //Read 1 byte from the magnetometer

    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);                                             //Enable I2C and set bytes
    wait(0.1);
    response=WriteReg(MPUREG_EXT_SENS_DATA_00|READ_FLAG, 0x00); 
    //ReadRegs(MPUREG_EXT_SENS_DATA_00,response,1);
    response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);                                      //Read I2C 
    deselect();
    return response;
    }
    
/****************************
*                            MAG_Read CALIBRATION
*   call this function to 
*   returns Factory Trim value
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/
void mpu9250_spi::AK8963_calib_Magnetometer()
    {
    select();
    uint8_t response[3];
    float data;
    int i;

    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);                           //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_ASAX);                                         //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x83);                                               //Read 3 bytes from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);                                             //Enable I2C and set bytes
    wait(0.1);
    //response[0]=WriteReg(MPUREG_EXT_SENS_DATA_01|READ_FLAG, 0x00);                    //Read I2C 
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,3);
    
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);                                      //Read I2C 
        for(i=0; i<3; i++) 
            {
            data=response[i];
            Magnetometer_ASA[i]=((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
            }
    deselect();
    }

/****************************
*                            MAG_Read DATA
*   call this function to 
*   returns Factory Trim value
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/   
void mpu9250_spi::AK8963_read_Magnetometer()
    {
    select();
    uint8_t response[7];
    int16_t bit_data;
    float data;
    int i;

    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);                           //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL);                                          //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);                                               //Read 6 bytes from the magnetometer

    wait(0.1);
    ReadRegs(MPUREG_EXT_SENS_DATA_00,response,7);                                       //must start your read from AK8963A register 0x03 and read seven bytes so that...
                                                                                        //upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
        for(i=0; i<3; i++) 
            {
            bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
            data=(float)bit_data;
            Magnetometer[i]=data*Magnetometer_ASA[i];
            }
    deselect();
    }

/****************************
*                            MAG_Read DATA
*   call this function to 
*   returns Factory Trim value
*   Line_1:
*   Line_2: 
*   Line_3: 
*   Line_4: 
*   Line_5: 
*
****************************/     
void mpu9250_spi::read_all()
    {
    select();
    uint8_t response[21];
    int16_t bit_data;
    float data;
    int i;

                                                                                        //Send I2C command at first
    WriteReg(MPUREG_I2C_SLV0_ADDR,AK8963_I2C_ADDR|READ_FLAG);                           //Set the I2C slave addres of AK8963 and set for read.
    WriteReg(MPUREG_I2C_SLV0_REG, AK8963_HXL);                                          //I2C slave 0 register address from where to begin data transfer
    WriteReg(MPUREG_I2C_SLV0_CTRL, 0x87);                                               //Read 7 bytes from the magnetometer
                                                                                        //must start your read from AK8963A register 0x03 and read seven bytes so that upon... 
                                                                                        //read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.

    wait(0.1);
    ReadRegs(MPUREG_ACCEL_XOUT_H,response,21);
    
        for(i=0; i<3; i++)                                                              //Get accelerometer value
            {
            bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
            data=(float)bit_data;
            accelerometer_data[i]=data/acc_divider;
            }
    
    bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];  //Get temperature
    data=(float)bit_data;
    Temperature=((data-21)/333.87)+21;
    
        for(i=4; i<7; i++)                                                              //Get gyroscop value
            {
            bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
            data=(float)bit_data;
            gyroscope_data[i-4]=data/gyro_divider;
            }
    
        for(i=7; i<10; i++)                                                             //Get Magnetometer value
            {
            bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
            data=(float)bit_data;
            Magnetometer[i-7]=data*Magnetometer_ASA[i-7];
            }
    deselect();
    }

/****************************
*                            SPI SELECT AND DESELECT
*   call this function to enable and disable mpu9250 communication bus
*   sets chip select H or L
*
****************************/

void mpu9250_spi::select()                                                              //Set CS low to start transmission (interrupts conversion)
    {
    cs = 0;
    }
        
void mpu9250_spi::deselect()                                                            //Set CS high to stop transmission (restarts conversion)
    {
    cs = 1;
    }