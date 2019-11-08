// Library made by Richard Mansell specifically for BPS.space
#ifndef IVN_user_H
#define IVN_user_H
#include "arduino.h"


/* Uncomment the Model being used */
//#define VNX VN100
//#define VNX VN200
#define VNX VN300

/* Uncomment the Serial port to VN being used */
//#define VSerial Serial
#define VSerial Serial1
//#define VSerial Serial2

/* Uncomment the SPI channel being used */
#define VSP SPI
//#define VSP SPI1
//#define VSP SPI2
//#define VSP SPI3

#endif