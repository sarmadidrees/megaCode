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
#define rightAngle         74
#define backAngle          350
#define leftAngle          200
#define Error              25       //error is changed for ROTATE (please change it for localization)

float    magneticX, magneticY, magneticZ;
float    headingAngle;
float    SetpointHeading;
char     orientation;

unsigned long previousMillis = 0;
const int interval = 10;

/********************************/
// For path finding 
#include "AstarPathFinder.h"

AstarPathFinder Astar = AstarPathFinder();

String command = "";
int counti = 0;
char expectOrien;
boolean followPath = false;
boolean doneRotate = false;
boolean doneStraight = false;
unsigned long pM = 0; //previous millis 

boolean rotateActive = false;
uint8_t agay = 0;

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
  nrf_send("Checking");
  
  initMagnetoMeter();
  
  inputString3.reserve(100);
  inputString2.reserve(100);
  recvString.reserve(100);
  command.reserve(100);
  
 /* prevHeading = frontAngle;
  newHeading = leftAngle;*/
  //TOD: add setpoint later

  Astar.initNodes();    //initializing A* path finding
}



void loop(void) {

/*
  readData();
  findONLYAngle();
  Serial.println(headingAngle);
*/

//unsigned long cM = millis(); 

if(followPath){
  if(Astar.pathFound()){
      if (doneRotate){
          readData();
          findHeadingAngle();
          
          if (orientation == expectOrien){
            if(Astar.stepCount()>counti){
              if(doneStraight){
                makeCommand();
                sendCommand();
                counti++;
                doneRotate = false;
                doneStraight = false;
              }
              else if (!doneStraight){
                //TODO: is jaga aye k front sonar pe kuch hai ya nahi?
                //if(noObstacle){
                //if(wiat is not ON)
                if(agay == 0){
                sendStraightCommand();
                agay = 1;
                }
                //else if (obstacle){ tell that there is obstacle and wait for command }
              }
            }
            else{
              followPath = false;
              doneRotate = false;
              doneStraight = false;
              counti = 0;
              nrf_send("Finished!!");
              agay = 0;
            }
          }
          else {
            sendCommand();
            doneRotate = false;
          }
        }
      
  }
  else{
    //get notification when path is not found correctly
  }
}


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
//  serial_receive();

  if(stringComplete3) { 
    nrf_send(inputString3);
    interperetMotorSerial();
    stringComplete3 = false;
    inputString3 = "";
  } 

  if(stringComplete2){
    nrf_send(inputString2);
    interperetSonarSerial();
    stringComplete2 = false;
    inputString2 = "";
  }
} // end loop()

/*CUSTOM functions*/
void serialEvent3() {
  while (Serial3.available() > 0 ) {
      char inChar = (char)Serial3.read();

      inputString3 += inChar;

      if(inChar == '\n'){
        stringComplete3 = true;
      }
  }
}

void serialEvent2() {
  while (Serial2.available() > 0 ) {
      char inChar = (char)Serial2.read();

      inputString2 += inChar;

      if(inChar == '\n'){
        stringComplete2 = true;
      }
  }
}

void interperetMotorSerial(){
  if(inputString3.startsWith("START")) rotateActive = true;
  else if(inputString3.startsWith("STOP")) rotateActive = false;
  else if (inputString3.startsWith("STRAIGHT")){ 
    doneStraight = true;
    agay = 0;
  }
  else if (inputString3.startsWith("ROTATE")) doneRotate = true;
}

void interperetSonarSerial(){
  
}

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
   
    if(recvString.startsWith("PATH")){
      //Command: PATH,currentX,currentY,endX,endY   //wrt actual plan
      
     // recvString = recvString.substring(5);
      planPath();
      //TODO: bool wala stuff k kis ko TRUE karna ha aur kis ko FALSE
    }
   
 else if(recvString.startsWith("M")){
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
    else if (recvString.startsWith("SONAR,L")){
      readData();
      findHeadingAngle();
      recvString = "L";
      recvString += ",";
      recvString += orientation; 
      Serial2.println(recvString);
    }
    else if (recvString.startsWith("SONAR,D")){
      recvString = recvString.substring(5);
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

void planPath(){
  //Command: PATH,currentX,currentY,endX,endY   //wrt actual plan
  int cX,cY;
  int eX,eY;
  int c1=1, c2=1; 
  Serial.println(recvString);
  c1 = recvString.indexOf(',')+1;
  c2 = recvString.indexOf(',',c1);
  cX = recvString.substring(c1,c2).toInt();
  
  c1 = c2+1;
  c2 = recvString.indexOf(',',c1);
  cY = recvString.substring(c1,c2).toInt();
  
  c1 = c2+1;
  c2 = recvString.indexOf(',',c1);
  eX = recvString.substring(c1,c2).toInt();
  
  c1 = c2+1;
  eY = recvString.substring(c1).toInt();

  Astar.Flush();
  Astar.findPath(cX,cY,eX,eY);
  Serial.println("aa");
  Serial.print(cX);Serial.print(",");Serial.print(cY);Serial.print(",");
  Serial.print(eX);Serial.print(",");Serial.println(eY);
  Serial.println(Astar.stepCount());
  followPath = true;
  doneRotate = true;
  doneStraight = true;
  counti = 0;
  
  readData();
  findHeadingAngle();
  expectOrien = orientation;
}


void makeCommand(){
    switch (orientation){
      case 'N':
        if(Astar.finalPath[counti].Y == Astar.finalPath[counti + 1].Y){
          if((Astar.finalPath[counti].X + 1) == Astar.finalPath[counti + 1].X){
            command = "C,X\n";
            expectOrien = 'N';
          }
          else if((Astar.finalPath[counti].X - 1) == Astar.finalPath[counti + 1].X){
            command = "C,L2\n";
            expectOrien = 'S';
          }
        }
        else if(Astar.finalPath[counti].X == Astar.finalPath[counti + 1].X){
            if((Astar.finalPath[counti].Y + 1) == Astar.finalPath[counti + 1].Y){
              command = "C,R1\n";
              expectOrien = 'E';
            }
            else if((Astar.finalPath[counti].Y - 1) == Astar.finalPath[counti + 1].Y){
              command = "C,L1\n";
              expectOrien = 'W';
            }
        }
      break;

      case 'E':
        if(Astar.finalPath[counti].Y == Astar.finalPath[counti + 1].Y){
          if((Astar.finalPath[counti].X + 1) == Astar.finalPath[counti + 1].X){
            command = "C,L1\n";
            expectOrien = 'N';
          }
          else if((Astar.finalPath[counti].X - 1) == Astar.finalPath[counti + 1].X){
            command = "C,R1\n";
            expectOrien = 'S';
          }
        }
        else if(Astar.finalPath[counti].X == Astar.finalPath[counti + 1].X){
            if((Astar.finalPath[counti].Y + 1) == Astar.finalPath[counti + 1].Y){
              command = "C,X\n";
              expectOrien = 'E';
            }
            else if((Astar.finalPath[counti].Y - 1) == Astar.finalPath[counti + 1].Y){
              command = "C,L2\n";
              expectOrien = 'W';
            }
        }
      break;

      case 'S':
        if(Astar.finalPath[counti].Y == Astar.finalPath[counti + 1].Y){
          if((Astar.finalPath[counti].X + 1) == Astar.finalPath[counti + 1].X){
            command = "C,L2\n";
            expectOrien = 'N';
          }
          else if((Astar.finalPath[counti].X - 1) == Astar.finalPath[counti + 1].X){
            command = "C,X\n";
            expectOrien = 'S';
          }
        }
        else if(Astar.finalPath[counti].X == Astar.finalPath[counti + 1].X){
            if((Astar.finalPath[counti].Y + 1) == Astar.finalPath[counti + 1].Y){
              command = "C,L1\n";
              expectOrien = 'E';
            }
            else if((Astar.finalPath[counti].Y - 1) == Astar.finalPath[counti + 1].Y){
              command = "C,R1\n";
              expectOrien = 'W';
            }
        }
      break;

      case 'W':
        if(Astar.finalPath[counti].Y == Astar.finalPath[counti + 1].Y){
          if((Astar.finalPath[counti].X + 1) == Astar.finalPath[counti + 1].X){
            command = "C,R1\n";
            expectOrien = 'N';
          }
          else if((Astar.finalPath[counti].X - 1) == Astar.finalPath[counti + 1].X){
            command = "C,L1\n";
            expectOrien = 'S';
          }
        }
        else if(Astar.finalPath[counti].X == Astar.finalPath[counti + 1].X){
            if((Astar.finalPath[counti].Y + 1) == Astar.finalPath[counti + 1].Y){
              command = "C,L2\n";
              expectOrien = 'E';
            }
            else if((Astar.finalPath[counti].Y - 1) == Astar.finalPath[counti + 1].Y){
              command = "C,X\n";
              expectOrien = 'W';
            }
        }
      break;
    }
}

void sendCommand(){
  if(command.startsWith("C,X")){
    doneRotate = true;
  }
  else
  Serial3.print(command);
  
  Serial.print(command);
}

void sendStraightCommand(){
  //bool wait = true;   //make this "wait" false when straightDone is recieved from MotorBoard
  Serial3.print("C,S\n");
  Serial.print("C,S\n");
}

void serial_receive(void){
  
  if (stringComplete2) { 
        /*// swap TX & Rx addr for writing
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
        
        // restore TX & Rx addr for reading  
             
        radio.openWritingPipe(pipes[0]);
        radio.openReadingPipe(1,pipes[1]);
        radio.startListening();
        */

        nrf_send(inputString2);
        stringComplete2 = false;  
        inputString2 = "";
  } // endif

    if (stringComplete3) { 
        // swap TX & Rx addr for writing
        /*
        radio.openWritingPipe(pipes[1]);
        radio.openReadingPipe(0,pipes[0]);  
        radio.stopListening();
        radio.write(inputString3.c_str(),inputString3.length());
        
        Serial.print("S:");  
        Serial.print(inputString3);          
        Serial.println();
        */

        nrf_send(inputString3);
        
        
        
        stringComplete3 = false;
       
        // restore TX & Rx addr for reading  
        /*     
        radio.openWritingPipe(pipes[0]);
        radio.openReadingPipe(1,pipes[1]);
        radio.startListening();
        */  
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

   if ((headingAngle >= 0) && (headingAngle <= Error + 10)){
      headingAngle += 360;  
   }
   
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

void nrf_send(String input){
  
    // swap TX & Rx addr for writing
    input.toCharArray(SendPayload,31);
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(0,pipes[0]);  
    radio.stopListening();
   // radio.write(inputString2.c_str(),inputString2.length());
    radio.write(&SendPayload,strlen(SendPayload));

    Serial.print("nRF Send = ");
    Serial.println(SendPayload);
    
  // restore TX & Rx addr for reading  
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
    radio.startListening();
}

