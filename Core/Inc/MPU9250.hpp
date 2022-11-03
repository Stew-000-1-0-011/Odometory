/*
 * MPU9250.h
 *
 *  Created on: Apr 2, 2022
 *      Author: MarkSherstan
 */
/*
 * Modified by Stew
 */

#ifndef SRC_MPU9250_H_
#define SRC_MPU9250_H_

// Libraries
#include <cstdint>
#include <cmath>
#include "spi.h"  //S what is SPI.h?

// Constants
#define RAD2DEG 57.2957795131

// Defines
#define WHO_AM_I_9250_ANS 0x71
#define WHO_AM_I          0x75
#define USER_CTRL         0x6A
#define PWR_MGMT_1        0x6B
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_XOUT_H      0x3B
#define READWRITE         0x80
#define CS_SELECT         0
#define CS_DESELECT       1
#define SPI_TIMOUT_MS     1000

// Structs
struct RawData
{
    int16_t ax, ay, az, gx, gy, gz;
};

struct ProcessedData
{
    float ax, ay, az, gx, gy, gz;
};

struct GyroCal
{
    float x, y, z;
};

struct Gravity
{
    float x{}, y{}, z{};
};

struct Quaternion
{
    float w{}, x{}, y{}, z{};

    friend constexpr Quaternion operator*(const Quaternion& l, const Quaternion& r) noexcept
    {
        return {
             l.w * r.w + l.x * r.x + l.y * r.y + l.z * r.z,
            -l.x * r.w + l.w * r.x + l.z * r.y - l.y * r.z,
            -l.y * r.w - l.z * r.x + l.w * r.y + l.x * r.z,
            -l.z * r.w + l.y * r.x - l.x * r.y + l.w * r.z
        };
    }

    constexpr Quaternion operator!() noexcept
    {
        return {w, -x, -y, -z};
    }
};

struct Attitude
{
    float r, p, y;
};

// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

class MPU9250
{
private:
    // Functions
    void REG_READ(uint8_t addr, uint8_t *pRxData, uint16_t RxSize);
    void REG_WRITE(uint8_t *pAddr, uint8_t *pVal);
    void writeGyroFullScaleRange(uint8_t gFSR);
    void writeAccFullScaleRange(uint8_t aFSR);
    void toggleCS();

    // Variables
    float aScaleFactor, gScaleFactor;
    SPI_HandleTypeDef *_pSPI;
    GPIO_TypeDef *_pCSport;
    uint16_t _CSpin;

    // Default values
    uint8_t _gFSR = GFSR_500DPS;
    uint8_t _aFSR = AFSR_4G;
    float _tau = 0.98;
    float _dt = 0.004;

    // Structs
    GyroCal gyroCal;
    Gravity gravity{};
    Quaternion local_to_global{};

    Attitude attitude;

public:
    // Init
    MPU9250(SPI_HandleTypeDef *pSPI, GPIO_TypeDef *pCSport, uint16_t CSpin);

    // Functions
    void calibrateGyro(uint16_t numCalPoints);
    void measureGravity(uint16_t numCalPoints);
    void calculateLocalToGlobalQuaternion();
    ProcessedData processData();
    Attitude calcAttitude();
    RawData readRawData();
    void changeToGlobal(ProcessedData& data);
    uint8_t begin();

    void setGyroFullScaleRange(uint8_t gFSR);
    void setAccFullScaleRange(uint8_t aFSR);
    void setDeltaTime(float dt);
    void setTau(float tau);
};

#endif /* SRC_MPU9250_H_ */
