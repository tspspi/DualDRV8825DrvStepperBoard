#ifndef __is_included__8200db71_5235_4eff_8b64_33cd89dc9936
#define __is_included__8200db71_5235_4eff_8b64_33cd89dc9936

#include "./i2c.h"

#ifdef __cplusplus
	extern "C" {
#endif

#define i2cDualStepper_FLAG__ENABLED		0x01
#define i2cDualStepper_FLAG__SLEEP			0x02

struct i2cDualStepper;
struct i2cDualStepper_VTBL;

enum i2cDualStepperError {
	i2cDualStepperErrorE_Ok					= 0,

	i2cDualStepperErrorE_InvalidParam,
};

typedef enum i2cDualStepperError (*i2cDualStepper_Release)(struct i2cDualStepper* lpSelf);

typedef enum i2cDualStepperError (*i2cDualStepper_GetAccelerationDeceleration)(struct i2cDualStepper* lpSelf, double* lpAcceleration, double* lpDeceleration);
typedef enum i2cDualStepperError (*i2cDualStepper_SetAccelerationDeceleration)(struct i2cDualStepper* lpSelf, double lpAcceleration, double lpDeceleration);
typedef enum i2cDualStepperError (*i2cDualStepper_GetVMax)(struct i2cDualStepper* lpSelf, double* lpVMax);
typedef enum i2cDualStepperError (*i2cDualStepper_SetVMax)(struct i2cDualStepper* lpSelf, double vMax);
typedef enum i2cDualStepperError (*i2cDualStepper_GetAlpha)(struct i2cDualStepper* lpSelf, double* lpAlpha);
typedef enum i2cDualStepperError (*i2cDualStepper_SetAlpha)(struct i2cDualStepper* lpSelf, double alpha);
typedef enum i2cDualStepperError (*i2cDualStepper_GetMicrosteps)(struct i2cDualStepper* lpSelf, unsigned int* lpSteps);
typedef enum i2cDualStepperError (*i2cDualStepper_SetMicrosteps)(struct i2cDualStepper* lpSelf, unsigned int steps);

typedef enum i2cDualStepperError (*i2cDualStepper_GetFault)(struct i2cDualStepper* lpSelf, uint8_t* lpFaultBits);

typedef enum i2cDualStepperError (*i2cDualStepper_RecalculateConstants)(struct i2cDualStepper* lpSelf);

typedef enum i2cDualStepperError (*i2cDualStepper_GetCommandQueueLength)(struct i2cDualStepper* lpSelf, unsigned long int* lpQueueLength, unsigned long int* lpQueueFree);

typedef enum i2cDualStepperError (*i2cDualStepper_Queue_Sync)(struct i2cDualStepper* lpSelf, unsigned long int channel);
typedef enum i2cDualStepperError (*i2cDualStepper_Queue_ConstantSpeed)(struct i2cDualStepper* lpSelf, unsigned long int channel, double speed);
typedef enum i2cDualStepperError (*i2cDualStepper_Queue_MoveAngularDistance)(struct i2cDualStepper* lpSelf, unsigned long int channel, double distance);
typedef enum i2cDualStepperError (*i2cDualStepper_Queue_AccelerateToSpeed)(struct i2cDualStepper* lpSelf, unsigned long int channel, double targetSpeed);
typedef enum i2cDualStepperError (*i2cDualStepper_Queue_Hold)(struct i2cDualStepper* lpSelf, unsigned long int channel);
typedef enum i2cDualStepperError (*i2cDualStepper_Queue_DisableDriver)(struct i2cDualStepper* lpSelf, unsigned long int channel);

typedef enum i2cDualStepperError (*i2cDualStepper_Exec_Sync)(struct i2cDualStepper* lpSelf, unsigned long int channel);
typedef enum i2cDualStepperError (*i2cDualStepper_Exec_ConstantSpeed)(struct i2cDualStepper* lpSelf, unsigned long int channel, double speed);
typedef enum i2cDualStepperError (*i2cDualStepper_Exec_MoveAngularDistance)(struct i2cDualStepper* lpSelf, unsigned long int channel, double distance);
typedef enum i2cDualStepperError (*i2cDualStepper_Exec_AccelerateToSpeed)(struct i2cDualStepper* lpSelf, unsigned long int channel, double targetSpeed);
typedef enum i2cDualStepperError (*i2cDualStepper_Exec_Hold)(struct i2cDualStepper* lpSelf, unsigned long int channel);
typedef enum i2cDualStepperError (*i2cDualStepper_Exec_DisableDriver)(struct i2cDualStepper* lpSelf, unsigned long int channel);

typedef enum i2cDualStepperError (*i2cDualStepper_EmergencyStop)(struct i2cDualStepper* lpSelf);
typedef enum i2cDualStepperError (*i2cDualStepper_EmergencyOff)(struct i2cDualStepper* lpSelf);

struct i2cDualStepper_VTBL {
	i2cDualStepper_Release							release;

	i2cDualStepper_GetAccelerationDeceleration		getAccelerationDeceleration;
	i2cDualStepper_SetAccelerationDeceleration		setAccelerationDeceleration;
	i2cDualStepper_GetVMax							getVMax;
	i2cDualStepper_SetVMax							setVMax;
	i2cDualStepper_GetAlpha							getAlpha;
	i2cDualStepper_SetAlpha							setAlpha;
	i2cDualStepper_GetMicrosteps					getMicrosteps;
	i2cDualStepper_SetMicrosteps					setMicrosteps;
	i2cDualStepper_GetFault							getFault;
	i2cDualStepper_RecalculateConstants				recalculateConstants;

	i2cDualStepper_GetCommandQueueLength			getCommandQueueLength;

	i2cDualStepper_Queue_Sync						queueSync;
	i2cDualStepper_Queue_ConstantSpeed				queueConstantSpeed;
	i2cDualStepper_Queue_MoveAngularDistance		queueMoveAngularDistance;
	i2cDualStepper_Queue_AccelerateToSpeed			queueAccelerateToSpeed;
	i2cDualStepper_Queue_Hold						queueHold;
	i2cDualStepper_Queue_DisableDriver				queueDisableDriver;

	i2cDualStepper_Exec_Sync						execSync;
	i2cDualStepper_Exec_ConstantSpeed				execConstantSpeed;
	i2cDualStepper_Exec_MoveAngularDistance			execMoveAngularDistance;
	i2cDualStepper_Exec_AccelerateToSpeed			execAcceerateToSpeed;
	i2cDualStepper_Exec_Hold						execHold;
	i2cDualStepper_Exec_DisableDriver				execDisableDriver;

	i2cDualStepper_EmergencyStop					emergencyStop;
	i2cDualStepper_EmergencyOff						emergencyOff;
};

struct i2cDualStepper {
	struct {
		void*											lpReserved;
		struct i2cDualStepper_VTBL*						vtbl;
	} obj;

	int						devAddr;
	struct busI2C*			i2cBus;

	/*
		Settings
	*/
	double					maxV;
	double					acceleration;
	double					deceleration;
	double					alpha;

	unsigned int			microsteps;
};

static enum i2cDualStepperError i2cDualStepper_Open(
	struct i2cDualStepper** lpStepperOut,
	uint8_t deviceAddress,
	struct i2cBus* lpBus
);

#ifdef __cplusplus
	} /* extern "C" */
#endif


#endif /* __is_included__8200db71_5235_4eff_8b64_33cd89dc9936 */
