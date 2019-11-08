// Library made by Richard Mansell specifically for BPS.space
#ifndef IVN_lib_H
#define IVN_lib_H
#include <arduino.h>
#include <SPI.h>

// Page numbers refer to VN-300 User Manual
// https://www.vectornav.com/docs/default-source/documentation/vn-300-documentation/vn-300-user-manual-(um005).pdf

/* VN Registers */
#define		VN100_REG_USR			0	// RW-User definable Tag p86
#define     VN100_REG_MODEL			1	// R-Model Number p87
#define     VN100_REG_HWREV			2	// R-Hardware Revision p88
#define     VN100_REG_SN			3	// R-Serial Number p89
#define     VN100_REG_FWVER			4	// R-Firmware Revision p90
#define     VN100_REG_SBAUD			5	// RW-Serial Baud Rate p91
#define     VN100_REG_ADOR			6	// RW-Async Data Output Type p92
#define     VN100_REG_ADOF			7	// RW-Async Data Output Frequency p94
#define     VN100_REG_YPR			8	// R-Yaw Pitch Roll p150
#define     VN100_REG_QTN			9	// R-Attitude Quaternion p151
#define     VN100_REG_QTM			10	
#define     VN100_REG_QTA			11
#define     VN100_REG_QTR			12
#define     VN100_REG_QMA			13
#define     VN100_REG_QAR			14
#define     VN100_REG_QMR			15  // R-Quaternion, Magnetic, Acceleration & Angular Rates p153
#define     VN100_REG_DCM			16
#define     VN100_REG_MAG			17	// R-Magnetic Measurements p154
#define     VN100_REG_ACC			18	// R-Acceleration Measurements p155
#define     VN100_REG_GYR			19	// R-Angular Rate Measurements p156
#define     VN100_REG_MAR			20	// R-Magnetic, Acceleration & Angular Measurements p157
#define     VN100_REG_REF			21	// RW-Magnetic and Gravity Reverence Vectors p175
#define     VN100_REG_SIG			22
#define     VN100_REG_HSI			23	// RW-Hard Soft Iron Magnetic Compensation p113
#define     VN100_REG_ATP			24
#define     VN100_REG_ACT			25	// Acceleration Compensation p114
#define     VN100_REG_RFR			26	// RW-Reference Frame Rotation p116
#define     VN100_REG_YMR			27	// R-Attitude solution Yaw, Pitch, Roll, Magnetic, Acceleration & Angular Rates p152
#define     VN100_REG_ACG			28
#define     VN_REG_ComProControl	30	// RW-Communication Protocol Control p98
#define     VN_REG_SyncControl		32	// RW-Synchronization Control p95
#define     VN_REG_SyncStatus		33	// RW-Synchronization Status p107
#define		VN_REG_VPEBasic			35	// RW-VPE Basic Control p160
#define		VN_REG_HSICalib			44	// RW-Magnetometer Calibration Control p170
#define		VN_REG_MagCalc			47	// R-Calculated Magnetometer Calibration p171
#define     VN_REG_IMUMeasure		54	// R-IMU Measurements p111
#define     VN_REG_GNSSconf			55	// RW-GNSS Configuration p127
#define     VN_REG_GNSS1Offset		57	// RW-GNSS Antenna A Offset p128
#define     VN_REG_GNSS1_LLA		58  // R-GNSS Solution-LLA 1 p123
#define     VN_REG_GNSS1_ECEF		59  // R-GNSS Solution-ECEF 1 p124
#define		VN_REG_INS_LLA			63	// R-INS Solution-LLA p162
#define		VN_REG_INS_ECEF			64	// R-INS Solution-ECEF p163
#define		VN_REG_INSBasicconf		67	// RW-INS Basic Configuration p168
#define		VN_REG_INSState_LLA		72	// R-INS State-LLA p166
#define		VN_REG_INSState_ECEF	73	// R-INS State-ECEF p167
#define		VN_REG_FilterBias		74	// RW-Startup Filter Bias Estimate p169
#define     VN_REG_BinaryOut1		75	// RW-Binary Output 1 p102
#define     VN_REG_BinaryOut2		76	// RW-Binary Output 2 p103
#define     VN_REG_BinaryOut3		77	// RW-Binary Output 3 p104
#define     VN_REG_DeltaTVdata		80	// R-Delta Theta and Delta Velocity Data p112
#define     VN_REG_DeltaTVconf		82	// RW-Delta Theta and Delta Velocity Configuration p118
#define     VN_REG_RefVectorConf	83	// RW-Reference Vector Configuration p176
#define     VN_REG_GyroComp			84	// RW-Gyro Compensation p115
#define     VN_REG_IMUFilter		85	// RW-IMU Filtering Configuration p117
#define     VN_REG_GNSSCompassH		86	// R-GNSS Compass Signal Health p134
#define     VN_REG_GNSS2Offset		93	// RW-GNSS2 Antenna B Offset p129
#define     VN_REG_GNSSEstBase		97	// R-GNSS Estimated Baseline p132
#define     VN_REG_GNSSCompassStart	98	// R-GNSS Compass Startup p133
#define     VN_REG_GNSSSync			100	// RW-GNSS Sync Configuration p131
#define     VN_REG_NMEAOut1			101	// RW-NMEA Output 1 p105
#define     VN_REG_NMEAOut2			102	// RW-NMEA Output 2 p106
#define     VN_REG_GNSS2_LLA		103 // R-GNSS Solution-LLA 2 p125
#define     VN_REG_GNSS2_ECEF		104 // R-GNSS Solution-ECEF 2 p126
#define     VN_REG_CompYPRTBAAR		239	// R-Yaw, Pitch, Roll, True Body Acceleration & Angular Rates p158
#define     VN_REG_CompMAAR			240	// R-Magnetic, Acceleration & Angular Rates p159


/* VN CommandID */		//typedef enum "VN100_CmdID"
#define		VN100_CmdID_ReadRegister			0x01
#define		VN100_CmdID_WriteRegister			0x02
#define		VN100_CmdID_WriteSettings			0x03
#define		VN100_CmdID_RestoreFactorySettings	0x04
#define		VN100_CmdID_Tare					0x05
#define		VN100_CmdID_Reset					0x06

/* VN System Error */	//typedef enum "VN100_Error"
#define		VN100_Error_None					0	// No error
#define		VN100_Error_HardFaultException		1   /* If this error occurs, then the firmware on the VN-300 has experienced a hard fault exception.  To recover from this error the processor will force a restart, and a discontinuity will occur in the serial output.  The processor will restart within 50 ms of a hard fault error. */
#define		VN100_Error_InputBufferOverflow		2   /* The processor’s serial input buffer has experienced an overflow.  The processor has a 256 character input buffer. */
#define		VN100_Error_InvalidChecksum			3	// The checksum for the received command was invalid.
#define		VN100_Error_InvalidCommand			4	// The user has requested an invalid command.
#define		VN100_Error_NotEnoughParameters		5	// The user did not supply the minimum number of required parameters for the requested command.
#define		VN100_Error_TooManyParameters		6	// The user supplied too many parameters for the requested command.
#define		VN100_Error_InvalidParameter		7	// The user supplied a parameter for the requested command which was invalid.
#define		VN100_Error_InvalidRegister			8	// An invalid register was specified.
#define		VN100_Error_UnauthorizedAccess		9	// The user does not have permission to write to this register.
#define		VN100_Error_WatchdogReset			10	// A watchdog reset has occurred.  In the event of a non-recoverable error the internal watchdog will reset the processor within 50 ms of the error.
#define		VN100_Error_OutputBufferOverflow	11	// The output buffer has experienced an overflow.  The processor has a 2048 character output buffer.
#define		VN100_Error_InsufficientBandwidth	12	// The baud rate is not high enough to support the requested asynchronous data output at the requested data rate.
#define		VN100_Error_ErrorListOverflow		255	// An overflow event has occurred on the system error buffer.

/* VN Asynchronous Data Output Register (ADOR) */	//typedef enum "VN100_ADORType"
#define		VN100_ADOR_OFF		0
#define		VN100_ADOR_YPR		1
#define		VN100_ADOR_QTN		2
#define		VN100_ADOR_QTM		3
#define		VN100_ADOR_QTA		4
#define		VN100_ADOR_QTR		5
#define		VN100_ADOR_QMA		6
#define		VN100_ADOR_QAR		7
#define		VN100_ADOR_QMR		8
#define		VN100_ADOR_DCM		9
#define		VN100_ADOR_MAG		10
#define		VN100_ADOR_ACC		11
#define		VN100_ADOR_GYR		12
#define		VN100_ADOR_MAR		13
#define		VN100_ADOR_YMR		14

/* VN Asynchronous Data Output Rate Register (ADOF)*/	//typedef enum "VN100_ADOFType"
#define		VN100_ADOF_1HZ		1
#define		VN100_ADOF_2HZ		2
#define		VN100_ADOF_4HZ		4
#define		VN100_ADOF_5HZ      5
#define		VN100_ADOF_10HZ     10
#define		VN100_ADOF_20HZ		20
#define		VN100_ADOF_25HZ		25
#define		VN100_ADOF_40HZ		40
#define		VN100_ADOF_50HZ     50
#define		VN100_ADOF_100HZ	100
#define		VN100_ADOF_200HZ	200

/* VN Serial Baud Rate Register */		//typedef enum "VN100_BaudType"
#define		VN100_Baud_9600		9600
#define		VN100_Baud_19200    19200
#define		VN100_Baud_38400    38400
#define		VN100_Baud_57600    57600
#define		VN100_Baud_115200   115200
#define		VN100_Baud_128000	128000
#define		VN100_Baud_230400   230400
#define		VN100_Baud_460800   460800
#define		VN100_Baud_921600   921600

/* VN Accelerometer Gain Type */		//typedef enum "VN100_AccGainType"
#define		VN100_AccGain_2G	0
#define		VN100_AccGain_6G	1

/* 32-bit Parameter Type */
typedef union {
	unsigned			longUInt;
	float				Float;
} VN100_Param;

#define VN100_SPI_BUFFER_SIZE	8	// Temporary SPI buffer size placeholder

/* SPI Response Packet */
typedef struct {
	unsigned char		ZeroByte;
	unsigned char		CmdID;
	unsigned char		RegID;
	unsigned char		ErrID;
	VN100_Param			Data[VN100_SPI_BUFFER_SIZE];
} VN100_SPI_Packet;

/* AsyncMode Serial Selection (Offset 0) p40 */
#define		SerialNone	0
#define		SerialOne	1
#define		SerialTwo	2
#define		SerialBoth	3

/* RateDivisor Hz Selection based on 400Hz VN300 (Offset 2) p40 */
#define		Rate1Hz		400
#define		Rate2Hz		200
#define		Rate4Hz		100
#define		Rate5Hz		80
#define		Rate8Hz		50
#define		Rate10Hz	40
#define		Rate16Hz	25
#define		Rate20Hz	20
#define		Rate25Hz	16
#define		Rate40Hz	10
#define		Rate50Hz	8
#define		Rate80Hz	5
#define		Rate100Hz	4
#define		Rate200Hz	2
#define		Rate400Hz	1

/* Configurable Binary Output Group Output Fields p39 */
typedef union {
	struct{						// Output Fields as bits
		boolean OutField0 : 1;	// Field 0
		boolean OutField1 : 1;	// Field 1
		boolean OutField2 : 1;	// Field 2
		boolean OutField3 : 1;	// Field 3
		boolean OutField4 : 1;	// Field 4
		boolean OutField5 : 1;	// Field 5
		boolean OutField6 : 1;	// Field 6
		boolean OutField7 : 1;	// Field 7
		boolean OutField8 : 1;	// Field 8
		boolean OutField9 : 1;	// Field 9
		boolean OutField10 : 1;	// Field 10
		boolean OutField11 : 1;	// Field 11
		boolean OutField12 : 1;	// Field 12
		boolean OutField13 : 1;	// Field 13
		boolean OutField14 : 1;	// Field 14
		boolean OutField15 : 1;	// Field 15
	};
	unsigned int OutFields;		// Output Fields as single int (2 bytes)
} Binary_OutGroupField_Type;

/* Configurable Binary Output Groups p39 */
typedef struct {									// Output Group as collections of Fields (OutFields)
		Binary_OutGroupField_Type OutGroupCommon;	// Group 1 Common
		Binary_OutGroupField_Type OutGroupTime;		// Group 2 Time
		Binary_OutGroupField_Type OutGroupIMU;		// Group 3 IMU
		Binary_OutGroupField_Type OutGroupGNSS1;	// Group 4 GNSS1
		Binary_OutGroupField_Type OutGroupAttitude;	// Group 5 Attitude
		Binary_OutGroupField_Type OutGroupINS;		// Group 6 INS
		Binary_OutGroupField_Type OutGroupGNSS2;	// Group 7 GNSS2
		Binary_OutGroupField_Type OutGroupX;		// Group Extension bit--normally 0
} Binary_OutGroup_Type;


/* Binary Serial Output Message */
typedef struct {
	unsigned char Binary_Message_Group;
	Binary_OutGroup_Type Binary_Message_Fields;
	String Binary_Message_Payload;
	unsigned char Binary_Message_CRC;
	unsigned char Binary_Step;
	unsigned char Binary_Number_Groups;
	unsigned char Binary_Number_Payload;
	unsigned char Binary_Payload_Num;
} Binary_Message_Type;

/* ASCII Serial Output Message */
typedef struct {
	String ASCII_Message_Payload;
	unsigned int ASCII_Message_CRC;
} ASCII_Message_Type;

/* VN Serial Output Message info */
typedef struct {
	ASCII_Message_Type VN_Message_ASCII;
	Binary_Message_Type VN_Message_Binary;
	int VN_Message_Status;
} VN_Message_Type;

/* Define Common Serial words */
#define SWriteSave	"VNWNV"	// Write Save Command p83
#define SWriteReg	"VNWRG"	// Write Register Command p83
#define SReadReg	"VNRRG"	// Read Register Command p83
#define SRestFact	"VNRFS"	// Restore Factory Settings Command p84
#define SReset		"VNRST"	// Reset Command p84
#define SFirmUpdate	"VNFWU"	// Firmware Update Command p84
#define SComPrompt	"VNCMD"	// Serial Command Prompt Command p85
#define SAsyncPause	"VNASY"	// Asynchronous Output Pause Command p85
#define SBinaryPoll	"VNBOM"	// Binary Output Poll Command p85
#define SStart		"$"		// Starting symbol for Serial message
#define SEnd		"*"		// Ending symbol for Serial message
#define SEndCRC		"*XX"	// Ending symbol with no CRC for Serial message
#define AsyncPause	0		// Setting for pausing Async Output for command SAsyncPause
#define AsyncResume	1		// Setting for resuming Async Output for command SAsyncPause




class IVN_IMU {
	//······················································································································
	//    Constructor: using hardware SPI
	//······················································································································
	public: IVN_IMU();
	
	// Functions for Startup & Configuration
	public: void begin();
	public: void begin(float GNSS1X,float GNSS1Y,float GNSS1Z);
	public: void begin(float GNSS1X,float GNSS1Y,float GNSS1Z,float GNSS2X,float GNSS2Y,float GNSS2Z);
	public: void begin(float GNSS1X,float GNSS1Y,float GNSS1Z,float GNSS2X,float GNSS2Y,float GNSS2Z, float uncertain1,float uncertain2,float uncertain3);
	public: void begin(float GNSSarray[]);
	
	// Functions for Serial commands
	public: void readSerialReg(int register2Read);
	public: void writeAsyncPause(int setting);
	public: void readPoll(int channel);
	public: void writeSerialReg(int register2Write, int data[]);
	public: void writeSerialReg(int register2Write, float data[]);
	public: void writeFactoryReset();
	public: void writeReset();
	public: void writeSave();
	public: void writeFirmUpdate();
	public: void writeSerialPrompt();
	public: void writeUserTag(char userTag[]);
	public: void checkVNSerial(char inChar, VN_Message_Type &VN_Message);
	
	// Functions for Calculating CRC
	public: unsigned char calculateCRC8(unsigned char data[], int length);
	public: unsigned short calculateCRC16(unsigned char data[], int length);
	
	// Functions for Groups and Fields
	public: Binary_OutGroup_Type clearGroup(Binary_OutGroup_Type Group2clear);
	public: void startPoll(int channel, int rateSelect, Binary_OutGroup_Type Group2start);
	public: void startBinOutput(int channel, int serialPort, int rateSelect, Binary_OutGroup_Type Group2start);
	};
#endif