/*
 * flight_control.h
 * Jason Shiuan
 */

const unsigned int FLOAT_SIZE = sizeof(float);
const unsigned int UINT_SIZE = sizeof(unsigned int);

const float HEADER = 0xDEADBEEF;

typedef struct flightControlInfo
{
	float header;
	float pitch;
	float roll;
	float throttle;
	float yaw;
	unsigned int pot1;
	unsigned int pot2;
	unsigned int button1;
	unsigned int button2;
	float footer;
};

const unsigned int INFO_SIZE = sizeof(flightControlInfo);

uint8_t calculateChecksum(uint8_t * infoPointer)
{
	uint8_t checksum = 0;

	for (int i = 0; i < INFO_SIZE - FLOAT_SIZE; i++)
	{
		checksum ^= *(infoPointer++);
	}

	return checksum;
}

