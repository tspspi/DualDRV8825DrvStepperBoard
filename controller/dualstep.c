#include <stdlib.h>

#include "./dualstep.h"

#include <stdint.h>
#include <math.h>

#include "./serdeshelper.h"

#ifdef __cplusplus
	extern "C" {
#endif

enum stepperI2cCommand {
	i2cCmd_GetAccelerateDecelerate		= 0x01,	/* 8 byte payload Slave -> Master + 1 Byte Status */
	i2cCmd_SetAccelerateDecelerate		= 0x02,	/* 8 Byte payload Master -> Slave */
	i2cCmd_GetVMax						= 0x03, /* 4 byte payload Slave -> Master + 1 Byte Status */
	i2cCmd_SetVMax						= 0x04,	/* 4 Byte payload Mater -> Slave */
	i2cCmd_GetAlpha						= 0x05,	/* 4 Byte payload Slave -> Master + 1 Byte Status */
	i2cCmd_SetAlpha						= 0x06,	/* 4 Byte payload Master -> Slave */
	i2cCmd_GetMicrostepping				= 0x07, /* 1 Byte payload Slave -> Master (2x 3 Bit) + 1 Byte Status */
	i2cCmd_SetMicrostepping				= 0x08, /* 1 Byte payload Master -> Slave */

	i2cCmd_GetFault						= 0x0E, /* 1 Byte payload Master -> Slave (IS status) */
	i2cCmd_RecalculateConstants			= 0x0F, /* Used to trigger recalculation of all constants (expensive operation; system should be stopped) */



	i2cCmd_GetCommandQueueSize			= 0x10,	/* Get size (first byte) and unused entires (second byte) of command queue + 1 Byte status */



	i2cCmd_Queue_Sync					= 0x20,	/* Sync. point; 1 Byte Channel */
	i2cCmd_Queue_ConstSpeed				= 0x21,	/* Constant speed; 1 Byte Channel; 4 Byte Speed */
	i2cCmd_Queue_MoveTo					= 0x22,	/* Move To (accelerated); 1 Byte Channel; 4 Byte Position */
	i2cCmd_Queue_ConstSpeedAccel		= 0x23,	/* Constant speed with acceleration/deceleration; 1 Byte channel; 4 Byte speed */
	i2cCmd_Queue_Hold					= 0x2E,	/* Hold position; 1 byte channel */
	i2cCmd_Queue_DisableDrv				= 0x2F,	/* Disable drivers; 1 byte channel (both have to be ordered to disable to be effective) */

	i2cCmd_Exec_Sync					= 0x30,	/* Sync. point; 1 Byte Channel */
	i2cCmd_Exec_ConstSpeed				= 0x31,	/* Constant speed; 1 Byte Channel; 4 Byte Speed */
	i2cCmd_Exec_MoveTo					= 0x32,	/* Move To (accelerated); 1 Byte Channel; 4 Byte Position */
	i2cCmd_Exec_ConstSpeedAccel			= 0x33,	/* Constant speed with acceleration/deceleration; 1 Byte channel; 4 Byte speed */
	i2cCmd_Exec_Hold					= 0x3E,	/* Hold position; 1 byte channel */
	i2cCmd_Exec_DisableDrv				= 0x3F,	/* Disable drivers; 1 byte channel (both have to be ordered to disable to be effective) */

	i2cCmd_EmergencyStop				= 0xFE,	/* Keeps motors engaged but stopped */
	i2cCmd_EmergencyOff					= 0xFF, /* Keeps motors disabled */
};


static enum i2cDualStepperError i2cDual_Release(
	struct i2cDualStepper* lpSelf
) {
	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }

	/*
		We do NOT stop, disengage drivers, etc. if not
		told explicitly!
	*/
	free(lpSelf);
	return i2cDualStepperErrorE_Ok;
}

static enum i2cDualStepperError i2cDual_GetAccelerationDeceleration(
	struct i2cDualStepper* lpSelf,
	unsigned long int dwChannel,
	double* lpAcceleration,
	double* lpDeceleration
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(dwChannel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2] = { i2cCmd_GetAccelerateDecelerate, (uint8_t)dwChannel };
	uint8_t resp[8];

	/* Perform query */
	i2e = i2cWriteRead(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req), resp, sizeof(resp));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	if(lpAcceleration != NULL) { (*lpAcceleration) = (double)deserializeIEEE754SingleFloat_4Bytes(&(resp[0])); }
	if(lpDeceleration != NULL) { (*lpDeceleration) = (double)deserializeIEEE754SingleFloat_4Bytes(&(resp[4])); }

	return i2cDualStepperErrorE_Ok;
}

static enum i2cDualStepperError i2cDual_SetAccelerationDeceleration(
	struct i2cDualStepper* lpSelf,
	unsigned long int dwChannel,
	double lpAcceleration,
	double lpDeceleration
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(dwChannel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2+8];
	req[0] = i2cCmd_SetAccelerateDecelerate;
	req[1] = (uint8_t)dwChannel;

	serializeIEEE754SingleFloat_4Bytes((float)lpAcceleration, &(req[2]));
	serializeIEEE754SingleFloat_4Bytes((float)lpDeceleration, &(req[2+4]));

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}

static enum i2cDualStepperError i2cDual_GetVMax(
	struct i2cDualStepper* lpSelf,
	unsigned long int dwChannel,
	double* lpVMax
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(dwChannel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2] = { i2cCmd_GetVMax, (uint8_t)dwChannel };
	uint8_t resp[4];

	/* Perform query */
	i2e = i2cWriteRead(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req), resp, sizeof(resp));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	if(lpVMax != NULL) { (*lpVMax) = (double)deserializeIEEE754SingleFloat_4Bytes(&(resp[0])); }

	return i2cDualStepperErrorE_Ok;
}

static enum i2cDualStepperError i2cDual_SetVMax(
	struct i2cDualStepper* lpSelf,
	unsigned long int dwChannel,
	double vMax
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(dwChannel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2+4];
	req[0] = i2cCmd_SetVMax;
	req[1] = (uint8_t)dwChannel;

	serializeIEEE754SingleFloat_4Bytes((float)vMax, &(req[2]));

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_GetAlpha(
	struct i2cDualStepper* lpSelf,
	unsigned long int dwChannel,
	double* lpAlpha
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(dwChannel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2] = { i2cCmd_GetAlpha, (uint8_t)dwChannel };
	uint8_t resp[4];

	/* Perform query */
	i2e = i2cWriteRead(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req), resp, sizeof(resp));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	if(lpAlpha != NULL) { (*lpAlpha) = (double)deserializeIEEE754SingleFloat_4Bytes(&(resp[0])); }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_SetAlpha(
	struct i2cDualStepper* lpSelf,
	unsigned long int dwChannel,
	double alpha
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(dwChannel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2+4];
	req[0] = i2cCmd_SetAlpha;
	req[1] = (uint8_t)dwChannel;

	serializeIEEE754SingleFloat_4Bytes((float)alpha, &(req[2]));

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_GetMicrosteps(
	struct i2cDualStepper* lpSelf,
	unsigned long int dwChannel,
	unsigned int* lpSteps
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(dwChannel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[1];
	uint8_t resp[1];
	req[0] = i2cCmd_SetAlpha;

	i2e = i2cWriteRead(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req), resp, sizeof(resp));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	if(lpSteps != NULL) { (*lpSteps) = resp[0]; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_SetMicrosteps(
	struct i2cDualStepper* lpSelf,
	unsigned long int dwChannel,
	unsigned int steps
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(dwChannel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2];
	req[0] = i2cCmd_SetMicrostepping;
	req[1] = steps;

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}

static enum i2cDualStepperError i2cDual_GetFault(
	struct i2cDualStepper* lpSelf,
	uint8_t* lpFaultBits
) {
	/* TODO */
	return i2cDualStepperErrorE_NotImplemented;
}
static enum i2cDualStepperError i2cDual_RecalculateConstants(
	struct i2cDualStepper* lpSelf,
	unsigned long int dwChannel
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(dwChannel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2];
	req[0] = i2cCmd_RecalculateConstants;
	req[1] = (uint8_t)dwChannel;

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}

static enum i2cDualStepperError i2cDual_GetCommandQueueLength(
	struct i2cDualStepper* lpSelf,
	unsigned long int* lpQueueLength,
	unsigned long int* lpQueueFree
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[1] = { i2cCmd_GetCommandQueueSize };
	uint8_t resp[2];

	/* Perform query */
	i2e = i2cWriteRead(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req), resp, sizeof(resp));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	if(lpQueueLength != NULL) { (*lpQueueLength) = resp[0]; }
	if(lpQueueFree != NULL) { (*lpQueueFree) = resp[1]; }

	return i2cDualStepperErrorE_Ok;
}

static enum i2cDualStepperError i2cDual_Queue_Sync(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel
) {
	/* TODO */
	return i2cDualStepperErrorE_NotImplemented;
}
static enum i2cDualStepperError i2cDual_Queue_ConstantSpeed(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel,
	double speed
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(channel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2+4];
	req[0] = i2cCmd_Queue_ConstSpeed;
	req[1] = (uint8_t)channel;

	serializeIEEE754SingleFloat_4Bytes((float)speed, &(req[2]));

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_Queue_MoveAngularDistance(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel,
	double distance
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(channel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2+4];
	req[0] = i2cCmd_Queue_MoveTo;
	req[1] = (uint8_t)channel;

	serializeIEEE754SingleFloat_4Bytes((float)distance, &(req[2]));

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_Queue_AccelerateToSpeed(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel,
	double targetSpeed
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(channel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2+4];
	req[0] = i2cCmd_Queue_ConstSpeedAccel;
	req[1] = (uint8_t)channel;

	serializeIEEE754SingleFloat_4Bytes((float)targetSpeed, &(req[2]));

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_Queue_Hold(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(channel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2];
	req[0] = i2cCmd_Queue_Hold;
	req[1] = (uint8_t)channel;

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_Queue_DisableDriver(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(channel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2];
	req[0] = i2cCmd_Queue_DisableDrv;
	req[1] = (uint8_t)channel;

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}

static enum i2cDualStepperError i2cDual_Exec_Sync(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel
) {
	/* TODO */
	return i2cDualStepperErrorE_NotImplemented;
}
static enum i2cDualStepperError i2cDual_Exec_ConstantSpeed(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel,
	double speed
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(channel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2+4];
	req[0] = i2cCmd_Exec_ConstSpeed;
	req[1] = (uint8_t)channel;

	serializeIEEE754SingleFloat_4Bytes((float)speed, &(req[2]));

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_Exec_MoveAngularDistance(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel,
	double distance
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(channel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2+4];
	req[0] = i2cCmd_Exec_MoveTo;
	req[1] = (uint8_t)channel;

	serializeIEEE754SingleFloat_4Bytes((float)distance, &(req[2]));

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_Exec_AccelerateToSpeed(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel,
	double targetSpeed
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(channel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2+4];
	req[0] = i2cCmd_Exec_ConstSpeedAccel;
	req[1] = (uint8_t)channel;

	serializeIEEE754SingleFloat_4Bytes((float)targetSpeed, &(req[2]));

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_Exec_Hold(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(channel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2];
	req[0] = i2cCmd_Exec_Hold;
	req[1] = (uint8_t)channel;

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_Exec_DisableDriver(
	struct i2cDualStepper* lpSelf,
	unsigned long int channel
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if(channel > 1) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[2];
	req[0] = i2cCmd_Exec_DisableDrv;
	req[1] = (uint8_t)channel;

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}

static enum i2cDualStepperError i2cDual_EmergencyStop(
	struct i2cDualStepper* lpSelf
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[1];
	req[0] = i2cCmd_EmergencyStop;

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}
static enum i2cDualStepperError i2cDual_EmergencyOff(
	struct i2cDualStepper* lpSelf
) {
	enum i2cError i2e;

	if(lpSelf == NULL) { return i2cDualStepperErrorE_InvalidParam; }

	uint8_t req[1];
	req[0] = i2cCmd_EmergencyOff;

	i2e = i2cWrite(lpSelf->i2cBus, lpSelf->devAddr, req, sizeof(req));
	if(i2e != i2cE_Ok) { return i2cDualStepperErrorE_IOError; }

	return i2cDualStepperErrorE_Ok;
}



static struct i2cDualStepper_VTBL defaultStepperVTBL = {
	&i2cDual_Release,
	&i2cDual_GetAccelerationDeceleration,
	&i2cDual_SetAccelerationDeceleration,
	&i2cDual_GetVMax,
	&i2cDual_SetVMax,
	&i2cDual_GetAlpha,
	&i2cDual_SetAlpha,
	&i2cDual_GetMicrosteps,
	&i2cDual_SetMicrosteps,
	&i2cDual_GetFault,
	&i2cDual_RecalculateConstants,

	&i2cDual_GetCommandQueueLength,

	&i2cDual_Queue_Sync,
	&i2cDual_Queue_ConstantSpeed,
	&i2cDual_Queue_MoveAngularDistance,
	&i2cDual_Queue_AccelerateToSpeed,
	&i2cDual_Queue_Hold,
	&i2cDual_Queue_DisableDriver,

	&i2cDual_Exec_Sync,
	&i2cDual_Exec_ConstantSpeed,
	&i2cDual_Exec_MoveAngularDistance,
	&i2cDual_Exec_AccelerateToSpeed,
	&i2cDual_Exec_Hold,
	&i2cDual_Exec_DisableDriver,

	&i2cDual_EmergencyStop,
	&i2cDual_EmergencyOff
};

enum i2cDualStepperError i2cDualStepper_Open(
	struct i2cDualStepper** lpStepperOut,
	uint8_t deviceAddress,
	struct busI2C* lpBus
) {
	struct i2cDualStepper* lpNew;

	if(lpStepperOut == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	(*lpStepperOut) = NULL;

	if(lpBus == NULL) { return i2cDualStepperErrorE_InvalidParam; }
	if((deviceAddress & 0x80) != 0) {
		return i2cDualStepperErrorE_InvalidParam;
	}

	lpNew = (struct i2cDualStepper*)malloc(sizeof(struct i2cDualStepper));
	if(lpNew == NULL) { return i2cDualStepperErrorE_OutOfMemory; }

	lpNew->obj.lpReserved = (void*)lpNew;
	lpNew->obj.vtbl = &defaultStepperVTBL;
	lpNew->devAddr = deviceAddress;
	lpNew->i2cBus = lpBus;

	/*
		Use our own functions to query current settings
	*/


	(*lpStepperOut) = lpNew;
	return i2cDualStepperErrorE_Ok;
}

#ifdef __cplusplus
	} /* extern "C" { */
#endif
