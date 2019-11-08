// Library made by Richard Mansell specifically for BPS.space
#include <IVN_lib.h>
#include <IVN_user.h>

IVN_IMU VN300;    // Initialization of VN300 IMU object

Binary_OutGroup_Type  VNoutGroup;  // Make varable for holding Group and Field settings
VN_Message_Type       VNoutMessage; // Make varable for holding VN Serial Message info

// Measurements for the GNSS antenna A & B with uncertainty values
float beginGNSSdata[9] = {1.0, 0.0, 1.0, -2.0, 0.0, 1.2, 0.035, 0.035, 0.35};


void setup() {
  Serial.begin(115200);         // Begin Serial to Computer
  Serial.println(__FILE__);     // Print file name
  VSerial.begin(VN100_Baud_115200);        // Begin Serial with IMU

  /* Several different options for the ".begin" command */
  VN300.begin();                                // Begin with just Async Output enabled
  //VN300.begin(1.0,0.0,1.0);                   // Begin with GNSS1 X, Y, & Z given-- in the example as 1 meter, 0 meters, & 1 meter
  //VN300.begin(1.0,0.0,1.0,-2.0,0.0,1.2);      // Begin with GNSS1 X, Y, & Z and GNSS2 X, Y, & Z given and default uncertainty of 0.0254
  //VN300.begin(1.0,0.0,1.0,-2.0,0.0,1.2, 0.035, 0.035, 0.35);     // Begin with GNSS1 X, Y, & Z and GNSS2 X, Y, & Z given and user defined uncertainty
  //VN300.begin(beginGNSSdata);                     // Begin with GNSS1 and uncertainty in array variable

  int comProControlArray[7] = {0, 0, 0, 0, 1, 0, 1}; // Setup Serial communication to use 8-bit CRC and error reporting p98
  VN300.writeSerialReg(VN_REG_ComProControl, comProControlArray);

  /* Example of how to setup a Polled Binary Output */
  VNoutGroup = VN300.clearGroup(VNoutGroup);    // Clear variable for new Polling command
  VNoutGroup.OutGroupCommon.OutField3 = true;   // Set Group1, Field3 (YawPitchRoll)
  VNoutGroup.OutGroupIMU.OutField4 = true;      // Set Group 3, Field4 (Temp)
  VN300.startPoll(1, Rate10Hz, VNoutGroup);     // Start Polled Binary Output 1 updating at 10Hz (rate doesn't really matter)

  
}

void loop() {
  /* Example of how to read a Polled Binary Output */
  VN300.readPoll(1);        // Read Binary Output 1--could also be changed to 2 or 3

 
  delay(100);
}
