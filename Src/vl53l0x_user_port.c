#include "vl53l0x_user_port.h"

//Sensor 1 parameters
VL53L0X_Error Status = VL53L0X_ERROR_NONE;
VL53L0X_Dev_t MyDevice;
VL53L0X_Dev_t *pMyDevice = &MyDevice;
VL53L0X_Version_t Version;
VL53L0X_Version_t *pVersion = &Version;
VL53L0X_DeviceInfo_t DeviceInfo;
int32_t status_int;

//sensor 1 measurements
VL53L0X_RangingMeasurementData_t RangingMeasurementData;
VL53L0X_RangingMeasurementData_t *pRangingMeasurementData =&RangingMeasurementData;

#if SENSOR_NUMBER>1
//Sensor 2 parameters
VL53L0X_Error Status2 = VL53L0X_ERROR_NONE;
VL53L0X_Dev_t MyDevice2;
VL53L0X_Dev_t *pMyDevice2 = &MyDevice2;
VL53L0X_Version_t Version2;
VL53L0X_Version_t *pVersion2 = &Version2;
VL53L0X_DeviceInfo_t DeviceInfo2;
int32_t status_int2;

//sensor 2 measurements
VL53L0X_RangingMeasurementData_t RangingMeasurementData2;
VL53L0X_RangingMeasurementData_t *pRangingMeasurementData2 =&RangingMeasurementData2;
#endif

#if SENSOR_NUMBER>2
//Sensor 3 parameters
VL53L0X_Error Status3 = VL53L0X_ERROR_NONE;
VL53L0X_Dev_t MyDevice3;
VL53L0X_Dev_t *pMyDevice3 = &MyDevice3;
VL53L0X_Version_t Version3;
VL53L0X_Version_t *pVersion3 = &Version3;
VL53L0X_DeviceInfo_t DeviceInfo3;
int32_t status_int3;

//sensor 3 measurements
VL53L0X_RangingMeasurementData_t RangingMeasurementData3;
VL53L0X_RangingMeasurementData_t *pRangingMeasurementData3 =&RangingMeasurementData3;
#endif
#if SENSOR_NUMBER>3
//Sensor 4 parameters
VL53L0X_Error Status4 = VL53L0X_ERROR_NONE;
VL53L0X_Dev_t MyDevice4;
VL53L0X_Dev_t *pMyDevice4 = &MyDevice4;
VL53L0X_Version_t Version4;
VL53L0X_Version_t *pVersion4 = &Version4;
VL53L0X_DeviceInfo_t DeviceInfo4;
int32_t status_int4;

//sensor 4 measurements
VL53L0X_RangingMeasurementData_t RangingMeasurementData4;
VL53L0X_RangingMeasurementData_t *pRangingMeasurementData4 =&RangingMeasurementData4;
#endif

