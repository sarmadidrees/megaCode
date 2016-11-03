#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>


// RF24 radio(CE,CSN);
RF24 radio(49,47);

const uint64_t pipes[2] = { 0xDEDEDEDEE8LL, 0xDEDEDEDEE4LL };

boolean stringComplete = false;  // whether the string is complete
static int dataBufferIndex = 0;
boolean stringOverflow = false;
char charOverflow = 0;

String inputString3 = "";
String inputString2 = "";
char RecvPayload[31] = "";
char serialBuffer[31] = "";
char SendPayload[31] = "";

String recvString = "";

bool stringComplete3 = false;
bool stringComplete2 = false;

/**********************************************/
/******** MAGNETO-METER **********/
#include <Wire.h>

#define MagnetoAddress     0x1E
#define MagnetoGainAddr    0x20
#define GaussConst         11
/*  Magnetometer gain can be varried between 1.3 to 8.1 
  *******************************************************
   MagnetoGainAddr  |   GaussConst   |   Gain
        0x20        |      1100      |  +/- 1.3
        0x40        |       855      |  +/- 1.9
        0x60        |       670      |  +/- 2.5
        0x80        |       450      |  +/- 4.0
        0xA0        |       400      |  +/- 4.7
        0xC0        |       330      |  +/- 5.6
        0xE0        |       230      |  +/- 8.1 
   ******************************************************/
   
#define frontAngle         135
#define rightAngle         50
#define backAngle          323
#define leftAngle          230
#define Error              35       //error is changed for ROTATE (please change it for localization)

float    magneticX, magneticY, magneticZ;
float    headingAngle;
float  SetpointHeading;
char     orientation;

unsigned long previousMillis = 0;
const int interval = 10;

void setup(void) {
  
 Serial.begin(115200); 
 Serial2.begin(115200); 
 Serial3.begin(115200);  
 Serial.println("TESTING");

  radio.begin();
  
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(70);
  
  radio.enableDynamicPayloads();
  radio.setRetries(15,15);
  radio.setCRCLength(RF24_CRC_16);

  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);  
  
  radio.startListening();
  radio.printDetails();

  Serial.println();
  Serial.println("RF Chat V01.0");
  delay(500);

  initMagnetoMeter();
  
  inputString3.reserve(100);
  inputString2.reserve(100);
  recvString.reserve(100);

 /* prevHeading = frontAngle;
  newHeading = leftAngle;*/
  //TOD: add setpoint later 
}

boolean rotateActive = false;


void loop(void) {

/*
  readData();
  findONLYAngle();
  Serial.println(headingAngle);
*/

if (rotateActive){
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    readData();
    findHeadingAngle();

  Serial3.print("H,");
  Serial3.print(headingAngle);
  Serial3.print(",");
  Serial3.println(SetpointHeading);
  
//  Serial.print(headingAngle);
//  Serial.print(",");
//  Serial.println(SetpointHeading);
  }
}
  nRF_receive();
  serial_receive();
  
} // end loop()

void serialEvent3() {
  while (Serial3.available() > 0 ) {
      char inChar = (char)Serial3.read();

      inputString3 += inChar;

      if(inChar == '\n'){
        stringComplete3 = true;
      }
  } // end while()
} // end serialEvent()

void serialEvent2() {
  while (Serial2.available() > 0 ) {
      char inChar = (char)Serial2.read();

      inputString2 += inChar;

      if(inChar == '\n'){
        stringComplete2 = true;
      }
  } // end while()
} // end serialEvent()


void nRF_receive(void) {
  int len = 0;
  if ( radio.available() ) {

        bool done = false;
        
        len = radio.getDynamicPayloadSize();
        radio.read(&RecvPayload,len);
        delay(20);
        
    RecvPayload[len] = 0; // null terminate string
    
    Serial.print("R:");
    Serial.println(RecvPayload);

    recvString = String(RecvPayload);
    if(recvString.startsWith("M")){
      if(recvString[2] == 'G'){
          readData();
          findHeadingAngle();
       
          rotateActive = true;
          recvString = recvString.substring(2);
          Serial3.println(recvString);
        }
      else{
        rotateActive = false;  
        recvString = recvString.substring(2);
        Serial3.println(recvString);
      }
    }
    else if (recvString.startsWith("S")){
      recvString = "S";
      recvString += ",";
      recvString += orientation; 
      Serial2.println(recvString);
    }

    recvString = "";
    RecvPayload[0] = 0;  // Clear the buffers
    for(int i =0; i<=31;i++){
      RecvPayload[i] = 0;
    }
    RecvPayload[0] = 0;  // Clear the buffers
  }  
}

void serial_receive(void){
  
  if (stringComplete2) { 
        // swap TX & Rx addr for writing
        inputString2.toCharArray(SendPayload,31);
        radio.openWritingPipe(pipes[1]);
        radio.openReadingPipe(0,pipes[0]);  
        radio.stopListening();
       // radio.write(inputString2.c_str(),inputString2.length());
        radio.write(&SendPayload,strlen(SendPayload));
        
        Serial.print("S:");  
        //Serial.print(inputString2);
        Serial.println(SendPayload);          
        Serial.println();
        stringComplete2 = false;
       
        // restore TX & Rx addr for reading  
             
        radio.openWritingPipe(pipes[0]);
        radio.openReadingPipe(1,pipes[1]);
        radio.startListening();
          
        inputString2 = "";
  } // endif

    if (stringComplete3) { 
        // swap TX & Rx addr for writing
        
        radio.openWritingPipe(pipes[1]);
        radio.openReadingPipe(0,pipes[0]);  
        radio.stopListening();
        radio.write(inputString3.c_str(),inputString3.length());
        
        Serial.print("S:");  
        Serial.print(inputString3);          
        Serial.println();

        if(inputString3.startsWith("START")) rotateActive =true;
        else if(inputString3.startsWith("STOP")) rotateActive = false;
        
        stringComplete3 = false;
       
        // restore TX & Rx addr for reading  
             
        radio.openWritingPipe(pipes[0]);
        radio.openReadingPipe(1,pipes[1]);
        radio.startListening();
          
        inputString3 = "";
  } // endif

  
} // end serial_receive()


/********** Functions for MAGNETO-METER ***********/
void initMagnetoMeter(){

  Wire.begin();

  // Enable Magnetometer HMC5883l
  Wire.beginTransmission(MagnetoAddress);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(1);

  // Gain setting
  Wire.beginTransmission(MagnetoAddress);
  Wire.write(0x01);
  Wire.write(MagnetoGainAddr);
  Wire.endTransmission();
}

void readData(){

  int X, Y, Z;
  
  // Tell the HMC what register to begin writing data into
  Wire.beginTransmission(MagnetoAddress);
  Wire.write(0x03); 
  Wire.endTransmission();
  
  // Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(MagnetoAddress, 6);

  // Wait around until enough data is available
  while (Wire.available() < 6);

  // Getting Raw values from HCM5883
   X  = Wire.read()<<8;          //MSB  x 
   X |= Wire.read();             //LSB  x
   Z  = Wire.read()<<8;          //MSB  z
   Z |= Wire.read();             //LSB  z
   Y  = Wire.read()<<8;          //MSB  y
   Y |= Wire.read();             //LSB  y

   // For scaled values
   magneticX = (X / GaussConst) ;
   magneticY = (Y / GaussConst) ;
   magneticZ = Z;

}

void findHeadingAngle(){

   float heading = atan2(magneticY, magneticX);

   // Correct for when signs are reversed.
   if(heading < 0)
      heading += 2*PI;
    
   if(heading > 2*PI)
      heading -= 2*PI;
   
   // Convert radians to degrees for readability.
   headingAngle = (heading * 180)/(PI);

   // For Orientation
   if ( (headingAngle <= frontAngle + Error) && (headingAngle >= frontAngle - Error) )
          { orientation = 'N'; 
          SetpointHeading = frontAngle;}
   else if ( (headingAngle <= rightAngle + Error) && (headingAngle >= rightAngle - Error) )
           { orientation = 'E'; 
           SetpointHeading = rightAngle;}
   else if ( (headingAngle <= backAngle + Error) && (headingAngle >= backAngle - Error) )
           { orientation = 'S'; 
           SetpointHeading = backAngle;}
   else if ( (headingAngle <= leftAngle + Error) && (headingAngle >= leftAngle - Error) )
           { orientation = 'W'; 
           SetpointHeading = leftAngle;}
   else 
   orientation = 'U';     
}


void findONLYAngle(){
  float heading = atan2(magneticY, magneticX);

   // Correct for when signs are reversed.
   if(heading < 0)
      heading += 2*PI;
    
   if(heading > 2*PI)
      heading -= 2*PI;
   
   // Convert radians to degrees for readability.
   headingAngle = (heading * 180)/(PI);
}

