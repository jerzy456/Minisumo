/*
 * vl53l0x_user_port.h
 *
 *  Created on: 23 sty 2017
 *      Author: jerzy
 */

#ifndef VL53L0X_USER_PORT_H_
#define VL53L0X_USER_PORT_H_



#endif /* VL53L0X_USER_PORT_H_ */

#define DEFAULT_SENS_ADDR 0x52
#define SENSOR_NUMBER 2
#define COMMS_TYPE 1
#define COMMS_SPEED_KHZ 100

#include "vl53l0x_api.h"
#include "I2C.h"

//Sensor 1 parameters

extern	VL53L0X_Error Status;
extern	VL53L0X_Dev_t MyDevice;
extern	VL53L0X_Dev_t *pMyDevice;
extern	VL53L0X_Version_t Version;
extern	VL53L0X_Version_t *pVersion;
extern	VL53L0X_DeviceInfo_t DeviceInfo;
extern	int32_t status_int;

//sensor 1 measurements
extern VL53L0X_RangingMeasurementData_t RangingMeasurementData;
extern VL53L0X_RangingMeasurementData_t *pRangingMeasurementData;
#if SENSOR_NUMBER>1
//Sensor 2 parameters
extern VL53L0X_Error Status2;
extern VL53L0X_Dev_t MyDevice2;
extern VL53L0X_Dev_t *pMyDevice2;
extern VL53L0X_Version_t Version2;
extern VL53L0X_Version_t *pVersion2;
extern VL53L0X_DeviceInfo_t DeviceInfo2;
extern int32_t status_int2;

//sensor 2 measurements
extern VL53L0X_RangingMeasurementData_t RangingMeasurementData2;
extern VL53L0X_RangingMeasurementData_t *pRangingMeasurementData2;
#endif
#if SENSOR_NUMBER>2
//Sensor 3 parameters
extern VL53L0X_Error Status3;
extern VL53L0X_Dev_t MyDevice3;
extern VL53L0X_Dev_t *pMyDevice3;
extern VL53L0X_Version_t Version3;
extern VL53L0X_Version_t *pVersion3;
extern VL53L0X_DeviceInfo_t DeviceInfo3;
extern int32_t status_int3;

//sensor 3 measurements
extern VL53L0X_RangingMeasurementData_t RangingMeasurementData3;
extern VL53L0X_RangingMeasurementData_t *pRangingMeasurementData3;
#endif

#if SENSOR_NUMBER>3
//Sensor 4 parameters
extern VL53L0X_Error Status4;
extern VL53L0X_Dev_t MyDevice4;
extern VL53L0X_Dev_t *pMyDevice4;
extern VL53L0X_Version_t Version4;
extern VL53L0X_Version_t *pVersion4;
extern VL53L0X_DeviceInfo_t DeviceInfo4;
extern int32_t status_int4;

//sensor 4 measurements
extern VL53L0X_RangingMeasurementData_t RangingMeasurementData4;
extern VL53L0X_RangingMeasurementData_t *pRangingMeasurementData4;
#endif

//User function

