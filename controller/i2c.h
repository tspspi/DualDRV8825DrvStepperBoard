#ifndef __is_included__049b8590_ef39_44a6_b235_de42b2293097
#define __is_included__049b8590_ef39_44a6_b235_de42b2293097

#include <stdint.h>

#ifdef __cplusplus
	extern "C" {
#endif

struct busI2C {
	int hFileDescriptor;
};

enum i2cError {
	i2cE_Ok					= 0,

	i2cE_InvalidParam,
	i2cE_OutOfMemory,
	i2cE_DevFailed,
	i2cE_IOError,
};

enum i2cError i2cOpen(
	struct busI2C** lpOut,
	char* devFile
);
enum i2cError i2cWrite(
	struct busI2C* lpBus,

	uint8_t devAddr,

	uint8_t* lpBuffer,
	unsigned long int dwBytes
);
enum i2cError i2cRead(
	struct busI2C* lpBus,

	uint8_t devAddr,

	uint8_t* lpBufferOut,
	unsigned long int dwBytes
);
enum i2cError i2cWriteRead(
	struct busI2C* lpBus,
	uint8_t devAddr,

	uint8_t* lpBufferTX,
	unsigned long int dwBytesTX,

	uint8_t* lpBufferRX,
	unsigned long int dwBytesRX
);
void i2cClose(
	struct busI2C* lpBus
);

#ifdef __cplusplus
	} /* extern "C" */
#endif


#endif /* __is_included__049b8590_ef39_44a6_b235_de42b2293097 */
