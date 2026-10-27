#include "BNO055.h"
#include "mbed.h"
#include "IMU.h"
#include <cstdint>
#include <ostream>
#include<iostream>

/*
We added these two functions since they're definitely beyond what we expect from you as recruits
I'd be impressed if you understood the math behind them.
Basically just know that quats are 4 axis (1 real, 3 imaginary) numbers that allow us 
to nicely talk about rotation, and the functions spit out pitch roll and yaw. 
*/

void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
{
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t w,x,y,z;

    dt[0] = BNO055_QUATERNION_W_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 8, false);
        w = dt[1] << 8 | dt[0];
        x = dt[3] << 8 | dt[2];
        y = dt[5] << 8 | dt[4];
        z = dt[7] << 8 | dt[6];

        result->w = (double)w / 16384.0f;
        result->x = (double)x / 16384.0f;
        result->y = (double)y / 16384.0f;
        result->z = (double)z / 16384.0f;
    } 
    else {
        cantReadDataCount++;
    }
}

void BNO055::get_angular_position_quat(IMU::EulerAngles *result){

    BNO055_QUATERNION_TypeDef q;
    get_quaternion(&q);

    float roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
    float pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
    float yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;

    memcpy(&result->roll, &roll, sizeof(float));
    memcpy(&result->pitch, &pitch, sizeof(float));
    memcpy(&result->yaw, &yaw, sizeof(float));
}

//constructors:

BNO055::BNO055(I2C &i2c, uint8_t addr) noexcept:
    _i2c{i2c},
    _i2c_p{&i2c},
    chip_addr{addr},
    _res{NC},//from PinNames.h, meaning no reset pin
    cantReadDataCount{}
{

};

BNO055::BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept //with reset pin
    :_i2c{i2c},
    _i2c_p{&i2c},
    chip_addr{addr},
    _res{p_reset},
    cantReadDataCount{}
    
{

};  
void BNO055::change_fusion_mode(uint8_t mode){
    //Change it into NDOF mode
    dt[0]=0x3D;
    dt[1]=mode;//the mode depend on the binary data written.
    int result=_i2c.write(chip_addr,dt,1);
    if(result){
        std::cout<<"Mode starting failed\n";
    }
    else{
        std::cout<<"Mode starting succeeded\n";
    }
}

void BNO055::init() noexcept{
    //cantreaddatacount=0----frequency----turn on power mode----wait----change to NDOF mode
    cantReadDataCount=0;
    _i2c.frequency(100000);//set frequency
    //BNO055::reset();

    //Turn on the power mode to normal
    dt[0]=PWR_MODE;
    dt[1]=0x00;//normal mode
    int result=_i2c.write(chip_addr,dt,1);
    if(result){
        std::cout<<"Failed";
    }
    else{
        std::cout<<"Success";
    }
    ThisThread::sleep_for(400ms);
    
    uint8_t mode=0b00001100;
    BNO055::change_fusion_mode(mode);

}

void BNO055::reset() noexcept{//reset data by writing the correct data to the correct register
    Timer timer;
    if(_res!=NC){
        dt[0]=SYS_TRIGGER; //register address
        dt[1]=0b00100000; //The information to put into the register, 6th position 1
        int result=_i2c.write(chip_addr,dt,1,false);//write one information into only
        if(result==1){
            std::cout<<"Reset Fail";
        }
        else{
            ThisThread::sleep_for(700ms);//wait for resetting
            while (timer.elapsed_time()<1000ms){ //if after 1s still cannot read the chip ID, return fail
                dt[0]=0x00;
                _i2c.write(chip_addr,dt,1);//to gain the chip ID
                _i2c.read(chip_addr,dt,1);
                if(dt[0]==0xA0){
                    std::cout<<"Reset Success";
                    return;
                }
            }
            std::cout<<"Reset Fail";
            return;
        }
    }
}

//Accessing Data
void BNO055::get_accel(BNO055_VECTOR_TypeDef *la){
    int16_t x,y,z;
    dt[0]=ACC_DATA_X_LSB;//register
    _i2c.write(chip_addr,dt,1);
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 6, false);//6 data input into dt
        x = dt[0] << 6 | dt[1];
        y = dt[2] << 6 | dt[3];
        z = dt[4] << 6 | dt[5];

        la->x = (double)x / 100.0f;
        la->y = (double)y / 100.0f;
        la->z = (double)z / 100.0f;
    } else {
        cantReadDataCount++;
    }
    
}

void BNO055::get_gyro(BNO055_VECTOR_TypeDef *gr){
    int16_t x,y,z;
    dt[0]=GYR_DATA_X_LSB;//register
    _i2c.write(chip_addr,dt,1);
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 6, false);//6 data input into dt
        x = dt[0] << 6 | dt[1];
        y = dt[2] << 6 | dt[3];
        z = dt[4] << 6 | dt[5];

        gr->x = (double)x / 900.0f;
        gr->y = (double)y / 900.0f;
        gr->z = (double)z / 900.0f;
    } else {
        cantReadDataCount++;
    }
}

