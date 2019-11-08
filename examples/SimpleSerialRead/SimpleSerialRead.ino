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

  
  /* Example of how to setup an Regular Binary Output */
  VNoutGroup = VN300.clearGroup(VNoutGroup);          // Clear variable for new Binary Output command
  VNoutGroup.OutGroupIMU.OutField6 = true;            // Set Group3, Field6 (DeltaTheta)
  VNoutGroup.OutGroupIMU.OutField7 = true;            // Set Group 3, Field7 (DeltaVel)
  VN300.startBinOutput(1, SerialOne, Rate20Hz, VNoutGroup);   // Start Async Binary Output 1 on VN-300 Serial 1, updating at 20Hz
}

void loop() {
  
  while (VSerial.available()) {                 // While Serial data is available from the VN
    char inChar = VSerial.read();               // Get next byte
    VN300.checkVNSerial(inChar, VNoutMessage);  // Check byte with Library
    if (VNoutMessage.VN_Message_Status==1){ // VN is not done and a Binary Message is being reaad
      
    } else if (VNoutMessage.VN_Message_Status==2){ // VN is not done and a ASCII Message is being read
      
    } else if (VNoutMessage.VN_Message_Status==3){ // VN is done and a Binary Message is ready
      // Do something with the Binary Message in VNoutMessage.VN_Message_Binary.Binary_Message_Payload
    } else if (VNoutMessage.VN_Message_Status==4){ // VN is done and a ASCII Message is ready
      // Do something with the ASCII Message in VNoutMessage.VN_Message_ASCII.ASCII_Message_Payload
    }
  }


}
