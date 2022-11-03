/*
 * MPU9250.cpp
 *
 *  Created on: Apr 2, 2022
 *      Author: MarkSherstan
 */

#include <cstdint>
#include <cstring>

namespace
{
    constexpr float eps = 0.000000001;

    // quote from cpprefjp.
    template<typename To, typename From>
    To bit_cast(const From& from) noexcept
    { // 実際には、さらに要件チェックが行われる
      // この実装ではTo型がデフォルト構築可能である必要があるが、C++20のbit_castでは必要ない
      To result;
      std::memcpy(&result, &from, sizeof(To));  // memcpyはconstexprではないため、
                                                // コンパイラが特殊な実装をする必要がある
      return result;
    }

    // quote C code of quick rsqrt from Wikipedia. (I translate it to C++.)
    float Q_rsqrt( float number )
    {

        std::int32_t i;
        float x2, y;
        const float threehalfs = 1.5F;

        x2 = number * 0.5F;
        y  = number;
        i = bit_cast<std::int32_t>(y);                       // evil floating point bit level hacking
        i  = 0x5f3759df - ( i >> 1 );               // what the fuck? 
        y  = bit_cast<float>(i);
        y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
    	y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

        return y;
    }
}

#include <MPU9250.hpp>

/// @brief MPU9250 SPI constructor
/// @param pSPI Pointer to SPI structure config
/// @param pCSport Pointer to GPIO CS port
/// @param CSpin GPIO pin number of CS pin
MPU9250::MPU9250(SPI_HandleTypeDef *pSPI, GPIO_TypeDef *pCSport, uint16_t CSpin)
{
    _pSPI = pSPI;
    _pCSport = pCSport;
    _CSpin = CSpin;
}

/// @brief Boot up the IMU and ensure we have a valid connection
/// @return Success [1] or fail [0]
uint8_t MPU9250::begin()
{
    // Initialize variables
    uint8_t check, addr, val;

    // Set attitude to zero conditions
    attitude.r = 0;
    attitude.p = 0;
    attitude.y = 0;

    // Confirm device
    REG_READ(WHO_AM_I, &check, 1);
    if (check == WHO_AM_I_9250_ANS)
    {
        // Startup / reset the sensor
        addr = PWR_MGMT_1;
        val = 0x00;
        REG_WRITE(&addr, &val);

        // Disable I2C (SPI only)
        addr = USER_CTRL;
        val = 0x10;
        REG_WRITE(&addr, &val);

        // Set the full scale ranges
        writeGyroFullScaleRange(_gFSR);
        writeAccFullScaleRange(_aFSR);
        return 1;
    }
    else
    {
        return 0;
    }
}

/// @brief Set the gyroscope full scale range
/// @param gFSR Desired yroscope full scale range
void MPU9250::setGyroFullScaleRange(uint8_t gFSR)
{
    _gFSR = gFSR;
}

/// @brief Set the accelerometer full scale range
/// @param aFSR Desired accelerometer full scale range
void MPU9250::setAccFullScaleRange(uint8_t aFSR)
{
    _aFSR = aFSR;
}

/// @brief Set the sampling duration (delta time) in seconds
/// @param dt Sampling time delta in seconds
void MPU9250::setDeltaTime(float dt)
{
    _dt = dt;
}

/// @brief Time constant of the complementary filter
/// @param tau Time constant
void MPU9250::setTau(float tau)
{
    _tau = tau;
}

/// @brief Toggle CS state to either start or end transmissions (default = high)
void MPU9250::toggleCS()
{
    HAL_GPIO_TogglePin(_pCSport, _CSpin);
}

/// @brief Read a specific registry address
/// @param pAddr Pointer to address to be written to
/// @param pVal Pointer of value to write at given address
void MPU9250::REG_WRITE(uint8_t *pAddr, uint8_t *pVal)
{
    toggleCS();
    HAL_SPI_Transmit(_pSPI, pAddr, 1, SPI_TIMOUT_MS);
    HAL_SPI_Transmit(_pSPI, pVal, 1, SPI_TIMOUT_MS);
    toggleCS();
}

/// @brief Read a specific registry address
/// @param addr Address to start reading at
/// @param pRxData Pointer to data buffer
/// @param RxSize Size of data buffer
void MPU9250::REG_READ(uint8_t addr, uint8_t *pRxData, uint16_t RxSize)
{
    toggleCS();
    uint8_t writeAddr = addr | READWRITE;
    HAL_SPI_Transmit(_pSPI, &writeAddr, 1, SPI_TIMOUT_MS);
    HAL_SPI_Receive(_pSPI, pRxData, RxSize, SPI_TIMOUT_MS);
    toggleCS();
}

/// @brief Set the gyroscope full scale range
/// @param gFSR Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s
void MPU9250::writeGyroFullScaleRange(uint8_t gFSR)
{
    // Variable init
    uint8_t addr = GYRO_CONFIG;
    uint8_t val;

    // Set the value
    switch (gFSR)
    {
    case GFSR_250DPS:
        gScaleFactor = 131.0;
        val = 0x00;
        REG_WRITE(&addr, &val);
        break;
    case GFSR_500DPS:
        gScaleFactor = 65.5;
        val = 0x08;
        REG_WRITE(&addr, &val);
        break;
    case GFSR_1000DPS:
        gScaleFactor = 32.8;
        val = 0x10;
        REG_WRITE(&addr, &val);
        break;
    case GFSR_2000DPS:
        gScaleFactor = 16.4;
        val = 0x18;
        REG_WRITE(&addr, &val);
        break;
    default:
        gScaleFactor = 65.5;
        val = 0x08;
        REG_WRITE(&addr, &val);
        break;
    }
}

/// @brief Set the accelerometer full scale range
/// @param aFSR Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g
void MPU9250::writeAccFullScaleRange(uint8_t aFSR)
{
    // Variable init
    uint8_t addr = ACCEL_CONFIG;
    uint8_t val;

    // Set the value
    switch (aFSR)
    {
    case AFSR_2G:
        aScaleFactor = 16384.0;
        val = 0x00;
        REG_WRITE(&addr, &val);
        break;
    case AFSR_4G:
        aScaleFactor = 8192.0;
        val = 0x08;
        REG_WRITE(&addr, &val);
        break;
    case AFSR_8G:
        aScaleFactor = 4096.0;
        val = 0x10;
        REG_WRITE(&addr, &val);
        break;
    case AFSR_16G:
        aScaleFactor = 2048.0;
        val = 0x18;
        REG_WRITE(&addr, &val);
        break;
    default:
        aScaleFactor = 8192.0;
        val = 0x08;
        REG_WRITE(&addr, &val);
        break;
    }
}

/// @brief Find offsets for each axis of gyroscope
void MPU9250::calibrateGyro(uint16_t numCalPoints)
{
    // Init
    RawData rawData;
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;

    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        rawData = readRawData();
        x += rawData.gx;
        y += rawData.gy;
        z += rawData.gz;
        HAL_Delay(3);
    }

    // Average the saved data points to find the gyroscope offset
    gyroCal.x = (float)x / (float)numCalPoints;
    gyroCal.y = (float)y / (float)numCalPoints;
    gyroCal.z = (float)z / (float)numCalPoints;
}

/// @brief stew: measure gravity.
void MPU9250::measureGravity(uint16_t numCalPoints)
{
    // Zero guard
    if (numCalPoints == 0)
    {
        numCalPoints = 1;
    }

    int32_t x{}, y{}, z{};
    // Save specified number of points
    for (uint16_t ii = 0; ii < numCalPoints; ii++)
    {
        const auto rawData = readRawData();
        x += rawData.ax;
        y += rawData.ay;
        z += rawData.az;
        HAL_Delay(3);
    }

    // Average the saved data points to find vartical direction
    gravity.x = (float)x / (float)numCalPoints;
    gravity.y = (float)y / (float)numCalPoints;
    gravity.z = (float)z / (float)numCalPoints;
}

/// @brief stew: calucrate quaternion which convert a local vector to global one
void MPU9250::calculateLocalToGlobalQuaternion()
{
    // vertical = - gravity
    // rotate is normalized vector which is orthogonal both to vertical and z axis.
    
    const auto tmp = gravity.x * gravity.x + gravity.y * gravity.y;  // please inline it when you read this code.
    const auto inv_gravity_norm = Q_rsqrt(tmp + gravity.z * gravity.z);  // assume it is sufficiently medium sized.
    const auto vertical_z_normed = - gravity.z * inv_gravity_norm;

    float sin_theta_2, cos_theta_2;

    if(1 - vertical_z_normed < eps)  // Q_rsqrt is week to too small number and too large number.
    {
        sin_theta_2 = 0;
        cos_theta_2 = 1;
    }
    else
    {
        sin_theta_2 = 1 / Q_rsqrt(1 - vertical_z_normed / 2);
        cos_theta_2 = 1 / Q_rsqrt(1 + vertical_z_normed / 2);
    }
    
    const auto inv_rotate_norm = Q_rsqrt(tmp);  // assume it is sufficiently medium sized.
    const auto rotate_x = - gravity.y * inv_rotate_norm;
    const auto rotate_y = gravity.x * inv_rotate_norm;

    local_to_global = { cos_theta_2, rotate_x * sin_theta_2, rotate_y * sin_theta_2, 0};
}

/// @brief Read raw data from IMU
/// @return Structure containing raw accelerometer and gyroscope data
RawData MPU9250::readRawData()
{
    // Data out and buffer init
    RawData rawData;
    uint8_t buf[14];

    // Subroutine for reading the raw data
    REG_READ(ACCEL_XOUT_H, &buf[0], 14);

    // Bit shift the data
    rawData.ax = buf[0] << 8 | buf[1];
    rawData.ay = buf[2] << 8 | buf[3];
    rawData.az = buf[4] << 8 | buf[5];
    // temperature = buf[6] << 8 | buf[7];
    rawData.gx = buf[8] << 8 | buf[9];
    rawData.gy = buf[10] << 8 | buf[11];
    rawData.gz = buf[12] << 8 | buf[13];

    // Result
    return rawData;
}

/// @brief Process the raw data into real world sensor values
/// @return Structure containing processed accelerometer and gyroscope data
ProcessedData MPU9250::processData()
{
    // Data out structure
    ProcessedData processedData;

    // Get raw values from the IMU
    RawData rawData = readRawData();

    // Convert accelerometer values to g's
    processedData.ax = rawData.ax / aScaleFactor;
    processedData.ay = rawData.ay / aScaleFactor;
    processedData.az = rawData.az / aScaleFactor;

    // Compensate for gyro offset
    processedData.gx = rawData.gx - gyroCal.x;
    processedData.gy = rawData.gy - gyroCal.y;
    processedData.gz = rawData.gz - gyroCal.z;

    // Convert gyro values to deg/s
    processedData.gx /= gScaleFactor;
    processedData.gy /= gScaleFactor;
    processedData.gz /= gScaleFactor;

    // Return structure
    return processedData;
}

/// @brief change data to global coordinate.
void MPU9250::changeToGlobal(ProcessedData& data) noexcept
{
    const auto modified_acc_data_quat = local_to_global * Quaternion{0, data.ax, data.ay, data.az} * !local_to_global;
    const auto modified_gyr_data_quat = local_to_global * Quaternion{0, data.gx, data.gy, data.gz} * !local_to_global;
    data = {modified_acc_data_quat.x, modified_acc_data_quat.y, modified_acc_data_quat.z, modified_gyr_data_quat.x, modified_gyr_data_quat.y, modified_gyr_data_quat.z};
}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter
/// @return Structure containing sensor attitude data
Attitude MPU9250::calcAttitude()
{
    // Read processed data
    ProcessedData sensorData = processData();

    // Complementary filter
    float accelPitch = atan2(sensorData.ay, sensorData.az) * RAD2DEG;
    float accelRoll = atan2(sensorData.ax, sensorData.az) * RAD2DEG;

    attitude.r = _tau * (attitude.r - sensorData.gy * _dt) + (1 - _tau) * accelRoll;
    attitude.p = _tau * (attitude.p - sensorData.gx * _dt) + (1 - _tau) * accelPitch;
    attitude.y += (sensorData.gz * _dt);

    // Return
    return attitude;
}
