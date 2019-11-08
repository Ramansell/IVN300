// Library made by Richard Mansell specifically for BPS.space
#include <IVN_lib.h>
#include <IVN_user.h>

IVN_IMU VN300;    // Initialization of VN300 IMU object

void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__);
  VN300.begin();                // Begin with just Async Output enabled
  //VN300.begin(1.0,0.0,1.0);     // Begin with GNSS1 X, Y, & Z given-- in the example as 1 meter, 0 meters, & 1 meter
  //VN300.begin(1.0,0.0,1.0,-2.0,0.0,1.2);     // Begin with GNSS1 X, Y, & Z and GNSS2 X, Y, & Z given and default uncertainty of 0.0254
  //VN300.begin(1.0,0.0,1.0,-2.0,0.0,1.2, 0.035, 0.035, 0.35);     // Begin with GNSS1 X, Y, & Z and GNSS2 X, Y, & Z given and user defined uncertainty
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
