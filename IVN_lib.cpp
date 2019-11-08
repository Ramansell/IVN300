// Library made by Richard Mansell specifically for BPS.space
#include "Arduino.h"
#include "IVN_user.h"
#include "IVN_lib.h"




IVN_IMU::IVN_IMU(void){
	int binaryData[4]={0,Rate1Hz,0,0};						// Sets data to be no Serial out, at 1Hz, 0 Groups, 0 Fields
	writeSerialReg(VN_REG_BinaryOut1, binaryData);			// Sets BinaryOutput1 to the above data=No Binary Output
	writeSerialReg(VN_REG_BinaryOut2, binaryData);			// Sets BinaryOutput2 to the above data=No Binary Output
	writeSerialReg(VN_REG_BinaryOut3, binaryData);			// Sets BinaryOutput3 to the above data=No Binary Output
	int binaryD[1]={0};
	writeSerialReg(VN100_REG_ADOR, binaryD);				// Sets Async Data Output to No Output
	writeAsyncPause(AsyncPause);							// Pauses all Async Outputs
	}
	
// Begins communication
void IVN_IMU::begin(void){
	writeAsyncPause(AsyncResume);							// Resumes all Async Outputs
}

// Begins communication with GNSS1 measurements
void IVN_IMU::begin(float GNSS1X,float GNSS1Y,float GNSS1Z){
	float data[3]={GNSS1X,GNSS1Y,GNSS1Z};					// Assign GNSS1 offsets to array
	writeSerialReg(VN_REG_GNSS1Offset, data);				// Write GNSS offsets to Register
	writeAsyncPause(AsyncResume);							// Resumes all Async Outputs
}

// Begins communication with GNSS1 & GNSS2 measurements with default uncertainty
void IVN_IMU::begin(float GNSS1X,float GNSS1Y,float GNSS1Z,float GNSS2X,float GNSS2Y,float GNSS2Z){
	float data[3]={GNSS1X,GNSS1Y,GNSS1Z};						// Assign GNSS1 offsets to array
	float data1[6]={GNSS2X,GNSS2Y,GNSS2Z, 0.0254, 0.0254, 0.0254};				// Assign GNSS2 offsets to array with default uncertainty
	writeSerialReg(VN_REG_GNSS1Offset,data);					// Write GNSS offsets to Register
	writeSerialReg(VN_REG_GNSS2Offset,data1);					// Write GNSS offsets to Register
	writeAsyncPause(AsyncResume);								// Resumes all Async Outputs
}

// Begins communication with GNSS1 & GNSS2 measurements with user uncertainty
void IVN_IMU::begin(float GNSS1X,float GNSS1Y,float GNSS1Z,float GNSS2X,float GNSS2Y,float GNSS2Z, float uncertain1,float uncertain2,float uncertain3){
	float data[3]={GNSS1X,GNSS1Y,GNSS1Z};						// Assign GNSS1 offsets to array
	float data1[6]={GNSS2X,GNSS2Y,GNSS2Z,  uncertain1, uncertain2, uncertain3};	// Assign GNSS2 offsets to array with user uncertainty
	writeSerialReg(VN_REG_GNSS1Offset,data);					// Write GNSS offsets to Register
	writeSerialReg(VN_REG_GNSS2Offset,data1);					// Write GNSS offsets to Register
	writeAsyncPause(AsyncResume);								// Resumes all Async Outputs
}

// Begins communication with GNSS1 & GNSS2 measurements with user uncertainty in array
void IVN_IMU::begin(float GNSSarray[]){
	float data[3];						// Assign GNSS1 offsets to array
	for(int i=0;i<3;i++){
		data[i]=GNSSarray[i];
	}
	float data1[6];	// Assign GNSS2 offsets to array with user uncertainty
	for(int i=0;i<6;i++){
		data1[i]=GNSSarray[i+3];
	}
	writeSerialReg(VN_REG_GNSS1Offset,data);					// Write GNSS offsets to Register
	writeSerialReg(VN_REG_GNSS2Offset,data1);					// Write GNSS offsets to Register
	writeAsyncPause(AsyncResume);								// Resumes all Async Outputs
}

// Checks Serial read to update messages, etc.
void IVN_IMU::checkVNSerial(char inChar, VN_Message_Type &VN_Message){  // Actual VN_Message from sketch is modified
	if(VN_Message.VN_Message_Status==0){	// If no message is being worked on
		if(inChar==0xFA){						// Found beginning of Binary Message
			VN_Message_Type Void_VN_Message;		// Make new blank VN_Message
			Void_VN_Message.VN_Message_Status=1;	// Sets status to "working on Binary Message"
			VN_Message=Void_VN_Message;				// Clear program VN_Message & add new status
					
		} else if (inChar==0x24){				// Found beginning of ASCII Message
			VN_Message_Type Void_VN_Message;		// Make new blank VN_Message
			Void_VN_Message.VN_Message_Status=2	;	// Sets status to "working on ASCII Message"
			VN_Message=Void_VN_Message;				// Clear program VN_Message & add new status
		}
		
	} else if(VN_Message.VN_Message_Status==1){		// If Binary message is being worked on
		if(VN_Message.VN_Message_Binary.Binary_Step==0){	// If first step in Binary Message
			VN_Message.VN_Message_Binary.Binary_Message_Group=inChar;	// Save Group byte
			VN_Message.VN_Message_Binary.Binary_Step+=1;				// move to next step in Binary Message
			for(int i=0;i<8;i++){										// count number of Groups
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group, i)==1){
					VN_Message.VN_Message_Binary.Binary_Number_Groups+=1;
				}
				VN_Message.VN_Message_Binary.Binary_Number_Groups*=2;	// double number of groups for all Field bytes to come next
			}
		} else if(VN_Message.VN_Message_Binary.Binary_Step==1 && bitRead(VN_Message.VN_Message_Binary.Binary_Number_Groups,0)==0){
			if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,0)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields=inChar;
			} else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,1)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields=inChar;
			} else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,2)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields=inChar;
			} else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,3)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields=inChar;
			} else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,4)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields=inChar;
			} else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,5)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields=inChar;
			} else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,6)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields=inChar;
			}
			
			VN_Message.VN_Message_Binary.Binary_Number_Groups-=1;
		} else if(VN_Message.VN_Message_Binary.Binary_Step==1 && bitRead(VN_Message.VN_Message_Binary.Binary_Number_Groups,0)==1){
			int tempInt=inChar;
			if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,0)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields+= (tempInt <<8);
				bitSet(VN_Message.VN_Message_Binary.Binary_Message_Group,0)=0;
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 0)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 1)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 2)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 3)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 4)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=16;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 5)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 6)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=24;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 7)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 8)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 9)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=24;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 10)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=20;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 11)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=28;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 12)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=2;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 13)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupCommon.OutFields, 14)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
						
			} else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,1)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields+= (tempInt <<8);
				bitSet(VN_Message.VN_Message_Binary.Binary_Message_Group,1)=0;
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields, 0)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields, 1)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields, 2)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields, 3)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=2;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields, 4)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields, 5)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields, 6)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields, 7)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields, 8)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupTime.OutFields, 9)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=1;}
				
			} else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,2)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields+= (tempInt <<8);
				bitSet(VN_Message.VN_Message_Binary.Binary_Message_Group,2)=0;
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 0)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=2;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 1)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 2)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 3)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 4)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 5)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 6)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=16;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 7)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 8)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 9)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupIMU.OutFields, 10)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				
			} else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,3)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields+= (tempInt <<8);
				bitSet(VN_Message.VN_Message_Binary.Binary_Message_Group,3)=0;
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 0)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 1)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 2)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=2;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 3)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=1;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 4)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=1;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 5)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=24;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 6)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=24;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 7)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 8)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 9)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 10)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 11)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 12)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=2;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS1.OutFields, 13)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=28;}
				
			} else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,4)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields+= (tempInt <<8);
				bitSet(VN_Message.VN_Message_Binary.Binary_Message_Group,4)=0;
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields, 0)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=2;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields, 1)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields, 2)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=16;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields, 3)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=36;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields, 4)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields, 5)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields, 6)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields, 7)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupAttitude.OutFields, 8)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				
			}  else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,5)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields+= (tempInt <<8);
				bitSet(VN_Message.VN_Message_Binary.Binary_Message_Group,5)=0;
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 0)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=2;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 1)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=24;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 2)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=24;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 3)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 4)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 5)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 6)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 7)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 8)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 9)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupINS.OutFields, 10)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				
			}  else if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Group,6)==1){
				VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields+= (tempInt <<8);
				bitSet(VN_Message.VN_Message_Binary.Binary_Message_Group,6)=0;
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 0)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 1)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=8;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 2)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=2;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 3)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=1;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 4)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=1;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 5)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=24;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 6)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=24;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 7)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 8)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 9)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=12;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 10)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 11)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=4;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 12)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=2;}
				if(bitRead(VN_Message.VN_Message_Binary.Binary_Message_Fields.OutGroupGNSS2.OutFields, 13)==1){VN_Message.VN_Message_Binary.Binary_Number_Payload+=28;}
			}
		
			
			VN_Message.VN_Message_Binary.Binary_Number_Groups-=1;
			if(VN_Message.VN_Message_Binary.Binary_Number_Groups==0){	// If all Groups have been checked
				VN_Message.VN_Message_Binary.Binary_Payload_Num=VN_Message.VN_Message_Binary.Binary_Number_Payload;	// Save number of bytes to be read next
				VN_Message.VN_Message_Binary.Binary_Step+=1;				// move to next step in Binary Message
			}
		} else if (VN_Message.VN_Message_Binary.Binary_Step==2 && VN_Message.VN_Message_Binary.Binary_Number_Payload>=0){		// If we are still working on the Payload
			VN_Message.VN_Message_Binary.Binary_Message_Payload+=inChar; // [VN_Message.VN_Message_Binary.Binary_Payload_Num-VN_Message.VN_Message_Binary.Binary_Number_Payload]
			VN_Message.VN_Message_Binary.Binary_Number_Payload-=1;
			
		} else if (VN_Message.VN_Message_Binary.Binary_Step==2 && VN_Message.VN_Message_Binary.Binary_Number_Payload<0){		// Last payload=CRC!
			VN_Message.VN_Message_Binary.Binary_Message_CRC=inChar; // Add character to CRC
			VN_Message.VN_Message_Binary.Binary_Step=3;				// Done with Binary Payload
			VN_Message.VN_Message_Status=3;							// Change status to Binary message is done!
		}
	} else if(VN_Message.VN_Message_Status==2){		// If ASCII message is being worked on
		if(inChar != 0x2A && VN_Message.VN_Message_ASCII.ASCII_Message_CRC != 'F'){		// If no end character and no 'F' in CRC
			VN_Message.VN_Message_ASCII.ASCII_Message_Payload+=inChar;							// Add character to Payload String
		} else if(inChar == 0x2A && VN_Message.VN_Message_ASCII.ASCII_Message_CRC != 'F'){	// If end character found and no 'F' in CRC
			VN_Message.VN_Message_ASCII.ASCII_Message_CRC='F';									// Add 'F' in CRC
		} else if(VN_Message.VN_Message_ASCII.ASCII_Message_CRC == 'F'){					// If 'F' found in CRC
			VN_Message.VN_Message_ASCII.ASCII_Message_Payload+=inChar;							// Add character to CRC
			VN_Message.VN_Message_Status=4;														// Change status to ASCII message is done!
		}
		
	}
}

/* Commands for Groups & Fields */
Binary_OutGroup_Type IVN_IMU::clearGroup(Binary_OutGroup_Type Group2clear){
	Group2clear.OutGroupCommon.OutFields=0;			// Group 1 Common
	Group2clear.OutGroupTime.OutFields=0;			// Group 2 Time
	Group2clear.OutGroupIMU.OutFields=0;			// Group 3 IMU
	Group2clear.OutGroupGNSS1.OutFields=0;			// Group 4 GNSS1
	Group2clear.OutGroupAttitude.OutFields=0;		// Group 5 Attitude
	Group2clear.OutGroupINS.OutFields=0;			// Group 6 INS
	Group2clear.OutGroupGNSS2.OutFields=0;			// Group 7 GNSS2
	Group2clear.OutGroupX.OutFields=0;				// Group Extension bit--normally 0
	return(Group2clear);
}
void IVN_IMU::startBinOutput(int channel, int serialPort, int rateSelect, Binary_OutGroup_Type Group2start){
	unsigned int GroupFieldsSelect[7]={0,0,0,0,0,0,0};	// variable for which Fields are selected for every Group
	int numDataArray=3;			// variable for number of elements needed in sendData array
	int GroupSelect=0;			// variable for which Groups have been selected
	
	if(Group2start.OutGroupCommon.OutFields>0){
		bitSet(GroupSelect,0);
		numDataArray+=1;
		if(Group2start.OutGroupCommon.OutField0==true){bitSet(GroupFieldsSelect[0],0);}
		if(Group2start.OutGroupCommon.OutField1==true){bitSet(GroupFieldsSelect[0],1);}
		if(Group2start.OutGroupCommon.OutField2==true){bitSet(GroupFieldsSelect[0],2);}
		if(Group2start.OutGroupCommon.OutField3==true){bitSet(GroupFieldsSelect[0],3);}
		if(Group2start.OutGroupCommon.OutField4==true){bitSet(GroupFieldsSelect[0],4);}
		if(Group2start.OutGroupCommon.OutField5==true){bitSet(GroupFieldsSelect[0],5);}
		if(Group2start.OutGroupCommon.OutField6==true){bitSet(GroupFieldsSelect[0],6);}
		if(Group2start.OutGroupCommon.OutField7==true){bitSet(GroupFieldsSelect[0],7);
		}
		if(Group2start.OutGroupCommon.OutField8==true){bitSet(GroupFieldsSelect[0],8);}
		if(Group2start.OutGroupCommon.OutField9==true){bitSet(GroupFieldsSelect[0],9);}
		if(Group2start.OutGroupCommon.OutField10==true){bitSet(GroupFieldsSelect[0],10);}
		if(Group2start.OutGroupCommon.OutField11==true){bitSet(GroupFieldsSelect[0],11);}
		if(Group2start.OutGroupCommon.OutField12==true){bitSet(GroupFieldsSelect[0],12);}
		if(Group2start.OutGroupCommon.OutField13==true){bitSet(GroupFieldsSelect[0],13);}
		if(Group2start.OutGroupCommon.OutField14==true){bitSet(GroupFieldsSelect[0],14);}
		if(Group2start.OutGroupCommon.OutField15==true){bitSet(GroupFieldsSelect[0],15);			
		}
	}
	if(Group2start.OutGroupTime.OutFields>0){
		bitSet(GroupSelect,1);
		numDataArray+=1;
		if(Group2start.OutGroupTime.OutField0==true){bitSet(GroupFieldsSelect[1],0);}
		if(Group2start.OutGroupTime.OutField1==true){bitSet(GroupFieldsSelect[1],1);}
		if(Group2start.OutGroupTime.OutField2==true){bitSet(GroupFieldsSelect[1],2);}
		if(Group2start.OutGroupTime.OutField3==true){bitSet(GroupFieldsSelect[1],3);}
		if(Group2start.OutGroupTime.OutField4==true){bitSet(GroupFieldsSelect[1],4);}
		if(Group2start.OutGroupTime.OutField5==true){bitSet(GroupFieldsSelect[1],5);}
		if(Group2start.OutGroupTime.OutField6==true){bitSet(GroupFieldsSelect[1],6);}
		if(Group2start.OutGroupTime.OutField7==true){bitSet(GroupFieldsSelect[1],7);
		}
		if(Group2start.OutGroupTime.OutField8==true){bitSet(GroupFieldsSelect[1],8);}
		if(Group2start.OutGroupTime.OutField9==true){bitSet(GroupFieldsSelect[1],9);}
		if(Group2start.OutGroupTime.OutField10==true){bitSet(GroupFieldsSelect[1],10);}
		if(Group2start.OutGroupTime.OutField11==true){bitSet(GroupFieldsSelect[1],11);}
		if(Group2start.OutGroupTime.OutField12==true){bitSet(GroupFieldsSelect[1],12);}
		if(Group2start.OutGroupTime.OutField13==true){bitSet(GroupFieldsSelect[1],13);}
		if(Group2start.OutGroupTime.OutField14==true){bitSet(GroupFieldsSelect[1],14);}
		if(Group2start.OutGroupTime.OutField15==true){bitSet(GroupFieldsSelect[1],15);
		}
	}
	if(Group2start.OutGroupIMU.OutFields>0){
		bitSet(GroupSelect,2);
		numDataArray+=1;
		if(Group2start.OutGroupIMU.OutField0==true){bitSet(GroupFieldsSelect[2],0);}
		if(Group2start.OutGroupIMU.OutField1==true){bitSet(GroupFieldsSelect[2],1);}
		if(Group2start.OutGroupIMU.OutField2==true){bitSet(GroupFieldsSelect[2],2);}
		if(Group2start.OutGroupIMU.OutField3==true){bitSet(GroupFieldsSelect[2],3);}
		if(Group2start.OutGroupIMU.OutField4==true){bitSet(GroupFieldsSelect[2],4);}
		if(Group2start.OutGroupIMU.OutField5==true){bitSet(GroupFieldsSelect[2],5);}
		if(Group2start.OutGroupIMU.OutField6==true){bitSet(GroupFieldsSelect[2],6);}
		if(Group2start.OutGroupIMU.OutField7==true){bitSet(GroupFieldsSelect[2],7);
		}
		if(Group2start.OutGroupIMU.OutField8==true){bitSet(GroupFieldsSelect[2],8);}
		if(Group2start.OutGroupIMU.OutField9==true){bitSet(GroupFieldsSelect[2],9);}
		if(Group2start.OutGroupIMU.OutField10==true){bitSet(GroupFieldsSelect[2],10);}
		if(Group2start.OutGroupIMU.OutField11==true){bitSet(GroupFieldsSelect[2],11);}
		if(Group2start.OutGroupIMU.OutField12==true){bitSet(GroupFieldsSelect[2],12);}
		if(Group2start.OutGroupIMU.OutField13==true){bitSet(GroupFieldsSelect[2],13);}
		if(Group2start.OutGroupIMU.OutField14==true){bitSet(GroupFieldsSelect[2],14);}
		if(Group2start.OutGroupIMU.OutField15==true){bitSet(GroupFieldsSelect[2],15);
		}
	}
	if(Group2start.OutGroupGNSS1.OutFields>0){
		bitSet(GroupSelect,3);
		numDataArray+=1;
		if(Group2start.OutGroupGNSS1.OutField0==true){bitSet(GroupFieldsSelect[3],0);}
		if(Group2start.OutGroupGNSS1.OutField1==true){bitSet(GroupFieldsSelect[3],1);}
		if(Group2start.OutGroupGNSS1.OutField2==true){bitSet(GroupFieldsSelect[3],2);}
		if(Group2start.OutGroupGNSS1.OutField3==true){bitSet(GroupFieldsSelect[3],3);}
		if(Group2start.OutGroupGNSS1.OutField4==true){bitSet(GroupFieldsSelect[3],4);}
		if(Group2start.OutGroupGNSS1.OutField5==true){bitSet(GroupFieldsSelect[3],5);}
		if(Group2start.OutGroupGNSS1.OutField6==true){bitSet(GroupFieldsSelect[3],6);}
		if(Group2start.OutGroupGNSS1.OutField7==true){bitSet(GroupFieldsSelect[3],7);
		}
		if(Group2start.OutGroupGNSS1.OutField8==true){bitSet(GroupFieldsSelect[3],8);}
		if(Group2start.OutGroupGNSS1.OutField9==true){bitSet(GroupFieldsSelect[3],9);}
		if(Group2start.OutGroupGNSS1.OutField10==true){bitSet(GroupFieldsSelect[3],10);}
		if(Group2start.OutGroupGNSS1.OutField11==true){bitSet(GroupFieldsSelect[3],11);}
		if(Group2start.OutGroupGNSS1.OutField12==true){bitSet(GroupFieldsSelect[3],12);}
		if(Group2start.OutGroupGNSS1.OutField13==true){bitSet(GroupFieldsSelect[3],13);}
		if(Group2start.OutGroupGNSS1.OutField14==true){bitSet(GroupFieldsSelect[3],14);}
		if(Group2start.OutGroupGNSS1.OutField15==true){bitSet(GroupFieldsSelect[3],15);
		}
	}
	if(Group2start.OutGroupAttitude.OutFields>0){
		bitSet(GroupSelect,4);
		numDataArray+=1;
		if(Group2start.OutGroupAttitude.OutField0==true){bitSet(GroupFieldsSelect[4],0);}
		if(Group2start.OutGroupAttitude.OutField1==true){bitSet(GroupFieldsSelect[4],1);}
		if(Group2start.OutGroupAttitude.OutField2==true){bitSet(GroupFieldsSelect[4],2);}
		if(Group2start.OutGroupAttitude.OutField3==true){bitSet(GroupFieldsSelect[4],3);}
		if(Group2start.OutGroupAttitude.OutField4==true){bitSet(GroupFieldsSelect[4],4);}
		if(Group2start.OutGroupAttitude.OutField5==true){bitSet(GroupFieldsSelect[4],5);}
		if(Group2start.OutGroupAttitude.OutField6==true){bitSet(GroupFieldsSelect[4],6);}
		if(Group2start.OutGroupAttitude.OutField7==true){bitSet(GroupFieldsSelect[4],7);
		}
		if(Group2start.OutGroupAttitude.OutField8==true){bitSet(GroupFieldsSelect[4],8);}
		if(Group2start.OutGroupAttitude.OutField9==true){bitSet(GroupFieldsSelect[4],9);}
		if(Group2start.OutGroupAttitude.OutField10==true){bitSet(GroupFieldsSelect[4],10);}
		if(Group2start.OutGroupAttitude.OutField11==true){bitSet(GroupFieldsSelect[4],11);}
		if(Group2start.OutGroupAttitude.OutField12==true){bitSet(GroupFieldsSelect[4],12);}
		if(Group2start.OutGroupAttitude.OutField13==true){bitSet(GroupFieldsSelect[4],13);}
		if(Group2start.OutGroupAttitude.OutField14==true){bitSet(GroupFieldsSelect[4],14);}
		if(Group2start.OutGroupAttitude.OutField15==true){bitSet(GroupFieldsSelect[4],15);
		}
	}
	if(Group2start.OutGroupINS.OutFields>0){
		bitSet(GroupSelect,5);
		numDataArray+=1;
		if(Group2start.OutGroupINS.OutField0==true){bitSet(GroupFieldsSelect[5],0);}
		if(Group2start.OutGroupINS.OutField1==true){bitSet(GroupFieldsSelect[5],1);}
		if(Group2start.OutGroupINS.OutField2==true){bitSet(GroupFieldsSelect[5],2);}
		if(Group2start.OutGroupINS.OutField3==true){bitSet(GroupFieldsSelect[5],3);}
		if(Group2start.OutGroupINS.OutField4==true){bitSet(GroupFieldsSelect[5],4);}
		if(Group2start.OutGroupINS.OutField5==true){bitSet(GroupFieldsSelect[5],5);}
		if(Group2start.OutGroupINS.OutField6==true){bitSet(GroupFieldsSelect[5],6);}
		if(Group2start.OutGroupINS.OutField7==true){bitSet(GroupFieldsSelect[5],7);
		}
		if(Group2start.OutGroupINS.OutField8==true){bitSet(GroupFieldsSelect[5],8);}
		if(Group2start.OutGroupINS.OutField9==true){bitSet(GroupFieldsSelect[5],9);}
		if(Group2start.OutGroupINS.OutField10==true){bitSet(GroupFieldsSelect[5],10);}
		if(Group2start.OutGroupINS.OutField11==true){bitSet(GroupFieldsSelect[5],11);}
		if(Group2start.OutGroupINS.OutField12==true){bitSet(GroupFieldsSelect[5],12);}
		if(Group2start.OutGroupINS.OutField13==true){bitSet(GroupFieldsSelect[5],13);}
		if(Group2start.OutGroupINS.OutField14==true){bitSet(GroupFieldsSelect[5],14);}
		if(Group2start.OutGroupINS.OutField15==true){bitSet(GroupFieldsSelect[5],15);
		}
	}
	if(Group2start.OutGroupGNSS2.OutFields>0){
		bitSet(GroupSelect,6);
		numDataArray+=1;
		if(Group2start.OutGroupGNSS2.OutField0==true){bitSet(GroupFieldsSelect[6],0);}
		if(Group2start.OutGroupGNSS2.OutField1==true){bitSet(GroupFieldsSelect[6],1);}
		if(Group2start.OutGroupGNSS2.OutField2==true){bitSet(GroupFieldsSelect[6],2);}
		if(Group2start.OutGroupGNSS2.OutField3==true){bitSet(GroupFieldsSelect[6],3);}
		if(Group2start.OutGroupGNSS2.OutField4==true){bitSet(GroupFieldsSelect[6],4);}
		if(Group2start.OutGroupGNSS2.OutField5==true){bitSet(GroupFieldsSelect[6],5);}
		if(Group2start.OutGroupGNSS2.OutField6==true){bitSet(GroupFieldsSelect[6],6);}
		if(Group2start.OutGroupGNSS2.OutField7==true){bitSet(GroupFieldsSelect[6],7);
		}
		if(Group2start.OutGroupGNSS2.OutField8==true){bitSet(GroupFieldsSelect[6],8);}
		if(Group2start.OutGroupGNSS2.OutField9==true){bitSet(GroupFieldsSelect[6],9);}
		if(Group2start.OutGroupGNSS2.OutField10==true){bitSet(GroupFieldsSelect[6],10);}
		if(Group2start.OutGroupGNSS2.OutField11==true){bitSet(GroupFieldsSelect[6],11);}
		if(Group2start.OutGroupGNSS2.OutField12==true){bitSet(GroupFieldsSelect[6],12);}
		if(Group2start.OutGroupGNSS2.OutField13==true){bitSet(GroupFieldsSelect[6],13);}
		if(Group2start.OutGroupGNSS2.OutField14==true){bitSet(GroupFieldsSelect[6],14);}
		if(Group2start.OutGroupGNSS2.OutField15==true){bitSet(GroupFieldsSelect[6],15);
		}
	}
	int sendData[numDataArray];
	sendData[0]=serialPort;
	sendData[1]=rateSelect;
	sendData[2]=GroupSelect;
	int nextSendData=3;
	if(GroupSelect>0){
		for(int i=0;i<7;i++){
			if(GroupFieldsSelect[i]>0){
				sendData[nextSendData]=GroupFieldsSelect[i];
				nextSendData+=1;
			}
		}
	}
	writeSerialReg(channel+74,sendData);
	writeAsyncPause(AsyncResume);								// Resumes all Async Outputs
}

void IVN_IMU::startPoll(int channel, int rateSelect, Binary_OutGroup_Type Group2start){
		unsigned int GroupFieldsSelect[7]={0,0,0,0,0,0,0};	// variable for which Fields are selected for every Group
		int numDataArray=3;			// variable for number of elements needed in sendData array
		int GroupSelect=0;			// variable for which Groups have been selected
		
		if(Group2start.OutGroupCommon.OutFields>0){
			bitSet(GroupSelect,0);
			numDataArray+=1;
			if(Group2start.OutGroupCommon.OutField0==true){bitSet(GroupFieldsSelect[0],0);}
			if(Group2start.OutGroupCommon.OutField1==true){bitSet(GroupFieldsSelect[0],1);}
			if(Group2start.OutGroupCommon.OutField2==true){bitSet(GroupFieldsSelect[0],2);}
			if(Group2start.OutGroupCommon.OutField3==true){bitSet(GroupFieldsSelect[0],3);}
			if(Group2start.OutGroupCommon.OutField4==true){bitSet(GroupFieldsSelect[0],4);}
			if(Group2start.OutGroupCommon.OutField5==true){bitSet(GroupFieldsSelect[0],5);}
			if(Group2start.OutGroupCommon.OutField6==true){bitSet(GroupFieldsSelect[0],6);}
			if(Group2start.OutGroupCommon.OutField7==true){bitSet(GroupFieldsSelect[0],7);
			}
			if(Group2start.OutGroupCommon.OutField8==true){bitSet(GroupFieldsSelect[0],8);}
			if(Group2start.OutGroupCommon.OutField9==true){bitSet(GroupFieldsSelect[0],9);}
			if(Group2start.OutGroupCommon.OutField10==true){bitSet(GroupFieldsSelect[0],10);}
			if(Group2start.OutGroupCommon.OutField11==true){bitSet(GroupFieldsSelect[0],11);}
			if(Group2start.OutGroupCommon.OutField12==true){bitSet(GroupFieldsSelect[0],12);}
			if(Group2start.OutGroupCommon.OutField13==true){bitSet(GroupFieldsSelect[0],13);}
			if(Group2start.OutGroupCommon.OutField14==true){bitSet(GroupFieldsSelect[0],14);}
			if(Group2start.OutGroupCommon.OutField15==true){bitSet(GroupFieldsSelect[0],15);
			}
		}
		if(Group2start.OutGroupTime.OutFields>0){
			bitSet(GroupSelect,1);
			numDataArray+=1;
			if(Group2start.OutGroupTime.OutField0==true){bitSet(GroupFieldsSelect[1],0);}
			if(Group2start.OutGroupTime.OutField1==true){bitSet(GroupFieldsSelect[1],1);}
			if(Group2start.OutGroupTime.OutField2==true){bitSet(GroupFieldsSelect[1],2);}
			if(Group2start.OutGroupTime.OutField3==true){bitSet(GroupFieldsSelect[1],3);}
			if(Group2start.OutGroupTime.OutField4==true){bitSet(GroupFieldsSelect[1],4);}
			if(Group2start.OutGroupTime.OutField5==true){bitSet(GroupFieldsSelect[1],5);}
			if(Group2start.OutGroupTime.OutField6==true){bitSet(GroupFieldsSelect[1],6);}
			if(Group2start.OutGroupTime.OutField7==true){bitSet(GroupFieldsSelect[1],7);
			}
			if(Group2start.OutGroupTime.OutField8==true){bitSet(GroupFieldsSelect[1],8);}
			if(Group2start.OutGroupTime.OutField9==true){bitSet(GroupFieldsSelect[1],9);}
			if(Group2start.OutGroupTime.OutField10==true){bitSet(GroupFieldsSelect[1],10);}
			if(Group2start.OutGroupTime.OutField11==true){bitSet(GroupFieldsSelect[1],11);}
			if(Group2start.OutGroupTime.OutField12==true){bitSet(GroupFieldsSelect[1],12);}
			if(Group2start.OutGroupTime.OutField13==true){bitSet(GroupFieldsSelect[1],13);}
			if(Group2start.OutGroupTime.OutField14==true){bitSet(GroupFieldsSelect[1],14);}
			if(Group2start.OutGroupTime.OutField15==true){bitSet(GroupFieldsSelect[1],15);
			}
		}
		if(Group2start.OutGroupIMU.OutFields>0){
			bitSet(GroupSelect,2);
			numDataArray+=1;
			if(Group2start.OutGroupIMU.OutField0==true){bitSet(GroupFieldsSelect[2],0);}
			if(Group2start.OutGroupIMU.OutField1==true){bitSet(GroupFieldsSelect[2],1);}
			if(Group2start.OutGroupIMU.OutField2==true){bitSet(GroupFieldsSelect[2],2);}
			if(Group2start.OutGroupIMU.OutField3==true){bitSet(GroupFieldsSelect[2],3);}
			if(Group2start.OutGroupIMU.OutField4==true){bitSet(GroupFieldsSelect[2],4);}
			if(Group2start.OutGroupIMU.OutField5==true){bitSet(GroupFieldsSelect[2],5);}
			if(Group2start.OutGroupIMU.OutField6==true){bitSet(GroupFieldsSelect[2],6);}
			if(Group2start.OutGroupIMU.OutField7==true){bitSet(GroupFieldsSelect[2],7);
			}
			if(Group2start.OutGroupIMU.OutField8==true){bitSet(GroupFieldsSelect[2],8);}
			if(Group2start.OutGroupIMU.OutField9==true){bitSet(GroupFieldsSelect[2],9);}
			if(Group2start.OutGroupIMU.OutField10==true){bitSet(GroupFieldsSelect[2],10);}
			if(Group2start.OutGroupIMU.OutField11==true){bitSet(GroupFieldsSelect[2],11);}
			if(Group2start.OutGroupIMU.OutField12==true){bitSet(GroupFieldsSelect[2],12);}
			if(Group2start.OutGroupIMU.OutField13==true){bitSet(GroupFieldsSelect[2],13);}
			if(Group2start.OutGroupIMU.OutField14==true){bitSet(GroupFieldsSelect[2],14);}
			if(Group2start.OutGroupIMU.OutField15==true){bitSet(GroupFieldsSelect[2],15);
			}
		}
		if(Group2start.OutGroupGNSS1.OutFields>0){
			bitSet(GroupSelect,3);
			numDataArray+=1;
			if(Group2start.OutGroupGNSS1.OutField0==true){bitSet(GroupFieldsSelect[3],0);}
			if(Group2start.OutGroupGNSS1.OutField1==true){bitSet(GroupFieldsSelect[3],1);}
			if(Group2start.OutGroupGNSS1.OutField2==true){bitSet(GroupFieldsSelect[3],2);}
			if(Group2start.OutGroupGNSS1.OutField3==true){bitSet(GroupFieldsSelect[3],3);}
			if(Group2start.OutGroupGNSS1.OutField4==true){bitSet(GroupFieldsSelect[3],4);}
			if(Group2start.OutGroupGNSS1.OutField5==true){bitSet(GroupFieldsSelect[3],5);}
			if(Group2start.OutGroupGNSS1.OutField6==true){bitSet(GroupFieldsSelect[3],6);}
			if(Group2start.OutGroupGNSS1.OutField7==true){bitSet(GroupFieldsSelect[3],7);
			}
			if(Group2start.OutGroupGNSS1.OutField8==true){bitSet(GroupFieldsSelect[3],8);}
			if(Group2start.OutGroupGNSS1.OutField9==true){bitSet(GroupFieldsSelect[3],9);}
			if(Group2start.OutGroupGNSS1.OutField10==true){bitSet(GroupFieldsSelect[3],10);}
			if(Group2start.OutGroupGNSS1.OutField11==true){bitSet(GroupFieldsSelect[3],11);}
			if(Group2start.OutGroupGNSS1.OutField12==true){bitSet(GroupFieldsSelect[3],12);}
			if(Group2start.OutGroupGNSS1.OutField13==true){bitSet(GroupFieldsSelect[3],13);}
			if(Group2start.OutGroupGNSS1.OutField14==true){bitSet(GroupFieldsSelect[3],14);}
			if(Group2start.OutGroupGNSS1.OutField15==true){bitSet(GroupFieldsSelect[3],15);
			}
		}
		if(Group2start.OutGroupAttitude.OutFields>0){
			bitSet(GroupSelect,4);
			numDataArray+=1;
			if(Group2start.OutGroupAttitude.OutField0==true){bitSet(GroupFieldsSelect[4],0);}
			if(Group2start.OutGroupAttitude.OutField1==true){bitSet(GroupFieldsSelect[4],1);}
			if(Group2start.OutGroupAttitude.OutField2==true){bitSet(GroupFieldsSelect[4],2);}
			if(Group2start.OutGroupAttitude.OutField3==true){bitSet(GroupFieldsSelect[4],3);}
			if(Group2start.OutGroupAttitude.OutField4==true){bitSet(GroupFieldsSelect[4],4);}
			if(Group2start.OutGroupAttitude.OutField5==true){bitSet(GroupFieldsSelect[4],5);}
			if(Group2start.OutGroupAttitude.OutField6==true){bitSet(GroupFieldsSelect[4],6);}
			if(Group2start.OutGroupAttitude.OutField7==true){bitSet(GroupFieldsSelect[4],7);
			}
			if(Group2start.OutGroupAttitude.OutField8==true){bitSet(GroupFieldsSelect[4],8);}
			if(Group2start.OutGroupAttitude.OutField9==true){bitSet(GroupFieldsSelect[4],9);}
			if(Group2start.OutGroupAttitude.OutField10==true){bitSet(GroupFieldsSelect[4],10);}
			if(Group2start.OutGroupAttitude.OutField11==true){bitSet(GroupFieldsSelect[4],11);}
			if(Group2start.OutGroupAttitude.OutField12==true){bitSet(GroupFieldsSelect[4],12);}
			if(Group2start.OutGroupAttitude.OutField13==true){bitSet(GroupFieldsSelect[4],13);}
			if(Group2start.OutGroupAttitude.OutField14==true){bitSet(GroupFieldsSelect[4],14);}
			if(Group2start.OutGroupAttitude.OutField15==true){bitSet(GroupFieldsSelect[4],15);
			}
		}
		if(Group2start.OutGroupINS.OutFields>0){
			bitSet(GroupSelect,5);
			numDataArray+=1;
			if(Group2start.OutGroupINS.OutField0==true){bitSet(GroupFieldsSelect[5],0);}
			if(Group2start.OutGroupINS.OutField1==true){bitSet(GroupFieldsSelect[5],1);}
			if(Group2start.OutGroupINS.OutField2==true){bitSet(GroupFieldsSelect[5],2);}
			if(Group2start.OutGroupINS.OutField3==true){bitSet(GroupFieldsSelect[5],3);}
			if(Group2start.OutGroupINS.OutField4==true){bitSet(GroupFieldsSelect[5],4);}
			if(Group2start.OutGroupINS.OutField5==true){bitSet(GroupFieldsSelect[5],5);}
			if(Group2start.OutGroupINS.OutField6==true){bitSet(GroupFieldsSelect[5],6);}
			if(Group2start.OutGroupINS.OutField7==true){bitSet(GroupFieldsSelect[5],7);
			}
			if(Group2start.OutGroupINS.OutField8==true){bitSet(GroupFieldsSelect[5],8);}
			if(Group2start.OutGroupINS.OutField9==true){bitSet(GroupFieldsSelect[5],9);}
			if(Group2start.OutGroupINS.OutField10==true){bitSet(GroupFieldsSelect[5],10);}
			if(Group2start.OutGroupINS.OutField11==true){bitSet(GroupFieldsSelect[5],11);}
			if(Group2start.OutGroupINS.OutField12==true){bitSet(GroupFieldsSelect[5],12);}
			if(Group2start.OutGroupINS.OutField13==true){bitSet(GroupFieldsSelect[5],13);}
			if(Group2start.OutGroupINS.OutField14==true){bitSet(GroupFieldsSelect[5],14);}
			if(Group2start.OutGroupINS.OutField15==true){bitSet(GroupFieldsSelect[5],15);
			}
		}
		if(Group2start.OutGroupGNSS2.OutFields>0){
			bitSet(GroupSelect,6);
			numDataArray+=1;
			if(Group2start.OutGroupGNSS2.OutField0==true){bitSet(GroupFieldsSelect[6],0);}
			if(Group2start.OutGroupGNSS2.OutField1==true){bitSet(GroupFieldsSelect[6],1);}
			if(Group2start.OutGroupGNSS2.OutField2==true){bitSet(GroupFieldsSelect[6],2);}
			if(Group2start.OutGroupGNSS2.OutField3==true){bitSet(GroupFieldsSelect[6],3);}
			if(Group2start.OutGroupGNSS2.OutField4==true){bitSet(GroupFieldsSelect[6],4);}
			if(Group2start.OutGroupGNSS2.OutField5==true){bitSet(GroupFieldsSelect[6],5);}
			if(Group2start.OutGroupGNSS2.OutField6==true){bitSet(GroupFieldsSelect[6],6);}
			if(Group2start.OutGroupGNSS2.OutField7==true){bitSet(GroupFieldsSelect[6],7);
			}
			if(Group2start.OutGroupGNSS2.OutField8==true){bitSet(GroupFieldsSelect[6],8);}
			if(Group2start.OutGroupGNSS2.OutField9==true){bitSet(GroupFieldsSelect[6],9);}
			if(Group2start.OutGroupGNSS2.OutField10==true){bitSet(GroupFieldsSelect[6],10);}
			if(Group2start.OutGroupGNSS2.OutField11==true){bitSet(GroupFieldsSelect[6],11);}
			if(Group2start.OutGroupGNSS2.OutField12==true){bitSet(GroupFieldsSelect[6],12);}
			if(Group2start.OutGroupGNSS2.OutField13==true){bitSet(GroupFieldsSelect[6],13);}
			if(Group2start.OutGroupGNSS2.OutField14==true){bitSet(GroupFieldsSelect[6],14);}
			if(Group2start.OutGroupGNSS2.OutField15==true){bitSet(GroupFieldsSelect[6],15);
			}
		}
		int sendData[numDataArray];
		sendData[0]=SerialNone;
		sendData[1]=rateSelect;
		sendData[2]=GroupSelect;
		int nextSendData=3;
		if(GroupSelect>0){
			for(int i=0;i<7;i++){
				if(GroupFieldsSelect[i]>0){
				sendData[nextSendData]=GroupFieldsSelect[i];
				nextSendData+=1;
				}
			}
		}
		writeSerialReg(channel+74,sendData);
		writeAsyncPause(AsyncResume);								// Resumes all Async Outputs
}

/* Commands for Serial Communication */

// Reads given Register
void IVN_IMU::readSerialReg(int register2Read){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SReadReg);		// Serial write Command
	VSerial.print(",");				// Serial write delimiter
	VSerial.print(register2Read);	// Serial write Register to be read
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

// Writes given data to given Register
void IVN_IMU::writeSerialReg(int register2Write, int data[]){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SWriteReg);		// Serial write Command
	VSerial.print(",");				// Serial write delimiter
	VSerial.print(register2Write);	// Serial write Register to be written to
	// Add sequence to write data to register
	for(int i=0;i<sizeof(data)/sizeof(data[0]);i++){
		VSerial.print(",");				// Serial write delimiter
		VSerial.print(data[i]);	// Serial write next data
	}
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

//Writes given data to given Register
void IVN_IMU::writeSerialReg(int register2Write, float data[]){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SWriteReg);		// Serial write Command
	VSerial.print(",");				// Serial write delimiter
	VSerial.print(register2Write);	// Serial write Register to be written to
	// Add sequence to write data to register
	for(int i=0;i<sizeof(data)/sizeof(data[0]);i++){
		VSerial.print(",");				// Serial write delimiter
		VSerial.print(data[i]);	// Serial write next data
	}
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

// Sends command to save settings to memory
void IVN_IMU::writeSave(){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SWriteSave);		// Serial write Command
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

// Sends command to restore to Factory Settings
void IVN_IMU::writeFactoryReset(){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SRestFact);		// Serial write Command
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

// Sends command to Reset Module
void IVN_IMU::writeReset(){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SReset);		// Serial write Command
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

// Sends command to update Firmware
void IVN_IMU::writeFirmUpdate(){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SFirmUpdate);		// Serial write Command
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

// Sends command to update Firmware
void IVN_IMU::writeSerialPrompt(){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SComPrompt);		// Serial write Command
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

// Sends command to pause/resume Asynchronous Output
void IVN_IMU::writeAsyncPause(int setting){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SComPrompt);		// Serial write Command
	VSerial.print(",");				// Serial write delimiter
	VSerial.print(setting);			// Serial write AsyncPause setting
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

// Sends command to read 1 of 3 Binary Outputs
void IVN_IMU::readPoll(int channel){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SBinaryPoll);		// Serial write Command
	VSerial.print(",");				// Serial write delimiter
	VSerial.print(channel);			// Serial write Binary Output channel to be read
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

// Sends command to write UserTag data to Register 0
void IVN_IMU::writeUserTag(char userTag[]){
	VSerial.print(SStart);			// Serial write "$"
	VSerial.print(SWriteReg);		// Serial write Command
	VSerial.print(",0,");			// Serial write delimiters & Register "0"
	// Add sequence to write userTag to register
	for(int i=0;i<sizeof(userTag)/sizeof(userTag[0]);i++){
		VSerial.print(userTag[i]);	// Serial write next data
	}
	VSerial.println(SEndCRC);			// Serial write ending with "XX" for CRC
}

// Calculates 8-bit CRC	
unsigned char IVN_IMU::calculateCRC8(unsigned char data[], int length){
	unsigned char cksum = 0;
	for(int i=0; i<length; i++){
		cksum ^= data[i];
	}
	return cksum;
}

// Calculates 16-bit CRC
unsigned short IVN_IMU::calculateCRC16(unsigned char data[], int length){
	unsigned short crc = 0;
	for(int i=0; i<length; i++){
		crc  = (unsigned char)(crc >> 8) | (crc << 8);
		crc ^= data[i];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0x00ff) << 5;
	}
	return crc;
}


	
