#include <sys/cdefs.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <dev/iicbus/iic.h>

#include "./i2c.h"

#ifdef __cplusplus
	extern "C" {
#endif


enum i2cError i2cOpen(
	struct busI2C** lpOut,
	char* devFile
) {
	struct busI2C* lpBus;

	if(lpOut == NULL) { return i2cE_InvalidParam; }
	(*lpOut) = NULL;

	if(devFile == NULL) { devFile = "/dev/iic1"; }

	lpBus = (struct busI2C*)malloc(sizeof(struct busI2C));
	if(lpBus == NULL) {
		return i2cE_OutOfMemory;
	}

	if((lpBus->hFileDescriptor = open(devFile, O_RDWR)) < 0) {
		free(lpBus);
		return i2cE_DevFailed;
	}

	(*lpOut) = lpBus;
	return i2cE_Ok;
}

enum i2cError i2cWrite(
	struct busI2C* lpBus,

	uint8_t devAddr,

	uint8_t* lpBuffer,
	unsigned long int dwBytes
) {
	struct iic_msg msg[1];
	struct iic_rdwr_data rdwr;

	if(lpBus == NULL) { return i2cE_InvalidParam; }
	if(dwBytes == 0) { return i2cE_Ok; }
	if(lpBuffer == NULL) { return i2cE_InvalidParam; }

	msg[0].slave = devAddr << 1;
	msg[0].flags = 0;
	msg[0].len = dwBytes;
	msg[0].buf = lpBuffer;

	rdwr.msgs = msg;
	rdwr.nmsgs = 1;

	if(ioctl(lpBus->hFileDescriptor, I2CRDWR, &rdwr) < 0) {
		return i2cE_IOError;
	}

	return i2cE_Ok;
}

enum i2cError i2cRead(
	struct busI2C* lpBus,

	uint8_t devAddr,

	uint8_t* lpBufferOut,
	unsigned long int dwBytes
) {
	struct iic_msg msg[1];
	struct iic_rdwr_data rdwr;

	if(lpBus == NULL) { return i2cE_InvalidParam; }
	if(dwBytes == 0) { return i2cE_Ok; }
	if(lpBufferOut == NULL) { return i2cE_InvalidParam; }

	msg[0].slave = devAddr << 1;
	msg[0].flags = IIC_M_RD;
	msg[0].len = dwBytes;
	msg[0].buf = lpBufferOut;

	rdwr.msgs = msg;
	rdwr.nmsgs = 1;

	if(ioctl(lpBus->hFileDescriptor, I2CRDWR, &rdwr) < 0) {
		return i2cE_IOError;
	}

	return i2cE_Ok;
}

enum i2cError i2cWriteRead(
	struct busI2C* lpBus,
	uint8_t devAddr,

	uint8_t* lpBufferTX,
	unsigned long int dwBytesTX,

	uint8_t* lpBufferRX,
	unsigned long int dwBytesRX
) {
	struct iic_msg msg[2];
	struct iic_rdwr_data rdwr;

	if(lpBus == NULL) { return i2cE_InvalidParam; }
	if((dwBytesTX == 0) && (dwBytesRX == 0)) { return i2cE_Ok; }
	if((lpBufferTX == NULL) || (lpBufferRX == NULL)) { return i2cE_InvalidParam; }

	msg[0].slave = devAddr << 1;
	msg[0].flags = 0;
	msg[0].len = dwBytesTX;
	msg[0].buf = lpBufferTX;

	msg[1].slave = devAddr << 1;
	msg[1].flags = IIC_M_RD;
	msg[1].len = dwBytesRX;
	msg[1].buf = lpBufferRX;

	rdwr.msgs = msg;
	rdwr.nmsgs = 2;

	if(ioctl(lpBus->hFileDescriptor, I2CRDWR, &rdwr) < 0) {
		return i2cE_IOError;
	}

	return i2cE_Ok;

}

void i2cClose(
	struct busI2C* lpBus
) {
	if(lpBus == NULL) { return; }

	close(lpBus->hFileDescriptor);
	free(lpBus);

	return;
}




#ifdef __cplusplus
	} /* extern "C" { */
#endif
