//=========================================================================
//Pixy Rover::Autonomous Rover
//Developed by: G. Sirlopu
//Portions of this code derived from the Pixy CMUCam5 pantilt example code
//========================================================================

#include "MotorDriver.h"
#include <SPI.h>  
#include <Pixy.h>

//declares the pixy camera object
Pixy pixy;

//MotorShield current sensing
const int SNS0 = A0;
const int SNS1 = A1;

char command = '\0';

#define X_CENTER 160L
#define Y_CENTER 100L
#define RCS_MIN_POS 0L
#define RCS_MAX_POS 1000L
#define RCS_CENTER_POS ((RCS_MAX_POS-RCS_MIN_POS)/2)

//---------------------------------------
// Servo Loop Class
// A Proportional/Derivative feedback
// loop for pan/tilt servo tracking of
// blocks.
// (Based on Pixy CMUcam5 example code)
//---------------------------------------
class ServoLoop
{

  public:
  ServoLoop(int32_t proportionalGain, int32_t derivativeGain);
  
  void update(int32_t error);
  
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_proportionalGain;
  int32_t m_derivativeGain;
  
};
  
// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{

  m_pos = RCS_CENTER_POS;
  m_proportionalGain = proportionalGain;
  m_derivativeGain = derivativeGain;
  m_prevError = 0x80000000L;

}
  
// ServoLoop Update
// Calculates new output based on the measured 
// error and the current state.
void ServoLoop::update(int32_t error)
{

  long int velocity;
  char buf[32];
  if (m_prevError!=0x80000000)
  {
    velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;
    m_pos += velocity;
    if (m_pos>RCS_MAX_POS)
    {
      m_pos = RCS_MAX_POS;
    }
    else if (m_pos<RCS_MIN_POS)
    {
      m_pos = RCS_MIN_POS;
    }
  }
  
  m_prevError = error;

}
// End Servo Loop Class
//---------------------------------------

ServoLoop panLoop(200,200); //servo loop for pan
ServoLoop tiltLoop(150,200); //servo loop for tilt

void setup()
{
  Serial.begin(9600);
  Serial.println("Starting serial communications...");
  
  /*Configure the motor A to control the wheel at the left side.*/
  /*Configure the motor B to control the wheel at the right side.*/
  pixy.init();
  motordriver.init();


}

uint32_t lastBlockTime = 0;

void loop()
{
  
  uint16_t blocks;
  blocks = pixy.getBlocks();
  
  char *help_menu = "\'e\' go forward\n\'d\' stop\n\'s\' left\n\'f\' right\n\'c\' go backward\n";       
  
  if (blocks)
  {
    int trackedBlock = TrackBlock(blocks);
    FollowBlock(trackedBlock);
    lastBlockTime = millis();
  }
  else if (millis() - lastBlockTime > 100)
  {
    motordriver.stop();
    Serial.println("can't find object");//motors.setLeftSpeed(0);
    //motors.setRightSpeed(0);
    ScanForBlocks();
  }
    
  if (Serial.available()>0)
  {
    command = Serial.read();
  }
  
  //overwrite commands in serial comms
  switch (command)
  {
    case 'e': //go forward
      Serial.println("Go!!\r\n"); 
      motordriver.goForward();       
      motordriver.setSpeed(200,MOTORB);
      motordriver.setSpeed(200,MOTORA);   
      command = '\0'; //reset command
      break;
    case 'c': //backward
      Serial.println("Back!!\r\n");     
      motordriver.goBackward();      
      motordriver.setSpeed(200,MOTORB);
      motordriver.setSpeed(200,MOTORA);     
      command = '\0'; //reset command
      break;   
    case 's': //left
      Serial.println("Left!!\r\n");     
      motordriver.goLeft();      
      motordriver.setSpeed(200,MOTORB);
      motordriver.setSpeed(200,MOTORA);     
      command = '\0'; //reset command
      break;    
    case 'f': //right
      Serial.println("Right!!\r\n");     
      motordriver.goRight();        
      motordriver.setSpeed(200,MOTORB);
      motordriver.setSpeed(200,MOTORA);    
      command = '\0'; //reset command
      break;           
    case 'd': //stop
      Serial.println("Stop!\r\n"); 
      motordriver.stop();
      command = '\0'; //reset command
      break;        
    case '\0':
      //do nothing
      break;    
    
    default:
      Serial.println(help_menu);
      command = '\0'; //reset command
      break; 
  }
  	
}

int oldX, oldY, oldSignature;
//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int TrackBlock(int blockCount)
{
  int trackedBlock = 0;
  long maxSize = 0;
//  Serial.print("blocks =");
//  Serial.println(blockCount);
  for (int i = 0; i < blockCount; i++)
  {
    if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
    {
      long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
      if (newSize > maxSize)
      {
        trackedBlock = i;
        maxSize = newSize;
      }
    }
  }
  
  int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
  int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;
  panLoop.update(panError);
  tiltLoop.update(tiltError);
  pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
  oldX = pixy.blocks[trackedBlock].x;
  oldY = pixy.blocks[trackedBlock].y;
  oldSignature = pixy.blocks[trackedBlock].signature;
  
  return trackedBlock;

}

//TODO:  REVISE FUNCTION TO MATCH CUSTOM ROVER/MOTOR SHIELD SETTINGS

//---------------------------------------
// Follow blocks via the custom Rover
//
// This code makes the robot base turn
// and move to follow the pan/tilt tracking
// of the head.
//---------------------------------------

int32_t size = 400;

void FollowBlock(int trackedBlock)
{
  int32_t followError = RCS_CENTER_POS - panLoop.m_pos; // How far off-center are we looking now?
  
  // Size is the area of the object.
  // We keep a running average of the last 8.
  size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height;
  size -= size >> 3;
  
  // Forward speed decreases as we approach the object (size is larger)
  int forwardSpeed = constrain(400 - (size/256), -100, 400);
  
  // Steering differential is proportional to the error times the forward speed
  int32_t differential = (followError + (followError * forwardSpeed))>>8;
  
  // Adjust the left and right speeds by the steering differential.
  int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
  int rightSpeed = constrain(forwardSpeed - differential, -400, 400);
  
  // And set the motor speeds
  motordriver.setSpeed(leftSpeed,MOTORB);
  motordriver.setSpeed(rightSpeed,MOTORA);
//  motors.setLeftSpeed(leftSpeed);
//  motors.setRightSpeed(rightSpeed);
}

//---------------------------------------
// Random search for blocks
//
// This code pans back and forth at random
// until a block is detected
//---------------------------------------

int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 150;
uint32_t lastMove = 0;
void ScanForBlocks()
{
  if (millis() - lastMove > 20)
  {
    lastMove = millis();
    panLoop.m_pos += scanIncrement;
    if ((panLoop.m_pos >= RCS_MAX_POS)||(panLoop.m_pos <= RCS_MIN_POS))
    {
      tiltLoop.m_pos = random(RCS_MAX_POS * 0.6, RCS_MAX_POS);
      scanIncrement = -scanIncrement;
    if (scanIncrement < 0)
    {
      motordriver.setSpeed(-250,MOTORB);
      motordriver.setSpeed(250,MOTORA);
//      motors.setLeftSpeed(-250);
//      motors.setRightSpeed(250);
    }
    else
    {
      motordriver.setSpeed(+180,MOTORB);
      motordriver.setSpeed(-180,MOTORA);
//      motors.setLeftSpeed(+180);
//      motors.setRightSpeed(-180);
    }
    delay(random(250, 500));
    }
    
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
  }
}

//
/////////////////////////////////////////
////     Motor Consumption Function    //
/////////////////////////////////////////
//
void MotorConsumption() {
  Serial.print("sensor 0: ");
  Serial.println(analogRead(SNS0));
  Serial.print("sensor 1: ");
  Serial.println(analogRead(SNS1));
  Serial.println("**************");
}
