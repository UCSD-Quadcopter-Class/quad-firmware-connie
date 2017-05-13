/*
 * flight_control.h
 * Jason Shiuan
 */

const unsigned int FLOAT_SIZE = sizeof(float);
const unsigned int UINT_SIZE = sizeof(unsigned int);
const unsigned int UCHAR_SIZE = sizeof(unsigned char);
const unsigned int FOOTER_SIZE = UCHAR_SIZE;

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
	unsigned char footer;
};

const unsigned int INFO_SIZE = sizeof(flightControlInfo);

unsigned char calculateChecksum(unsigned char * infoPointer)
{
	unsigned char checksum = 0;

	for (int i = 0; i < INFO_SIZE - FOOTER_SIZE; i++)
	{
		checksum ^= *(infoPointer++);
	}

	return checksum;
}

