#include <cstring>

#include <can.h>

#include <MPU9250.hpp>

#include <CRSLib/include/std_int.hpp>
#include <CRSLib/Can/RM0008/include/can_manager.hpp>
#include <CRSLib/Can/RM0008/include/filter_manager.hpp>


extern "C" CAN_HandleTypeDef hcan;

using namespace CRSLib::IntegerTypes;
MPU9250 imu{&hspi1, GPIOB, GPIO_PIN_6};
CRSLib::Can::RM0008::CanManager can_manager{&hcan};
constexpr u32 can_id_accelaration_xy = 0x100;
constexpr u32 can_id_gyro_z = 0x101;

extern "C" void main_cpp()
{
	imu.setGyroFullScaleRange(GFSR_500DPS);
	imu.setAccFullScaleRange(AFSR_4G);
	imu.setDeltaTime(0.004);
	imu.setTau(0.98);

	// Check if IMU configured properly and block if it didn't
	if (imu.begin() != 1)
	{
		while (1){}
	}

	// Calibrate the IMU
	imu.calibrateGyro(1500);
	imu.measureGravity(1500);
    imu.calculateLocalToGlobalQuaternion();

	// // Start timer and put processor into an efficient low power mode
	// HAL_TIM_Base_Start_IT(&htim11);
	// HAL_PWR_EnableSleepOnExit();
	// HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

	CRSLib::Can::RM0008::FilterManager::dynamic_initialize();
	HAL_CAN_Start(&hcan);

	while(true)
	{
		auto data = imu.processData();
		imu.changeToGlobal(data);

		CRSLib::Can::RM0008::TxFrame acc_xy;
		std::memcpy(acc_xy.data.data(), &data, sizeof(float) * 2);
		acc_xy.header.dlc = sizeof(float) * 2;

		CRSLib::Can::RM0008::TxFrame gyr_z;
		std::memcpy(gyr_z.data.data(), &data.gz, sizeof(float));
		gyr_z.header.dlc = sizeof(float);

		if(can_manager.pillarbox.not_full()) can_manager.pillarbox.post(can_id_accelaration_xy, acc_xy);
		if(can_manager.pillarbox.not_full()) can_manager.pillarbox.post(can_id_gyro_z, gyr_z);
	}

}
