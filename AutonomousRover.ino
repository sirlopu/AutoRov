#include "MotorDriver.h"
#include <SPI.h>  
#include <Pixy.h>

Pixy pixy;

//MotorShield current sensing
const int SNS0 = A0;
const int SNS1 = A1;

char command = '\0';


void setup()
{
  Serial.begin(9600);
  Serial.println("Starting serial communications...");
  
  /*Configure the motor A to control the wheel at the left side.*/
  /*Configure the motor B to control the wheel at the right side.*/
  pixy.init();
  motordriver.init();


}
 
void loop()
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32]; 
  char *help_menu = "\'e\' go forward\n\'d\' stop\n\'s\' left\n\'f\' right\n\'c\' go backward\n";       
  
  blocks = pixy.getBlocks();
  if (blocks)
  {
    i++;
    
    if (i%50==0)
    {
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        pixy.blocks[j].print();
      }
    }
  }  
  
  if (Serial.available()>0)
  {
    command = Serial.read();
  }
  
  
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
