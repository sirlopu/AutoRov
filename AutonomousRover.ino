#include <SPI.h>
#include <WiFi.h>

const int SNS0 = A0;
const int SNS1 = A1;

const int A_DIRECTION = 33;
const int A_BRAKE = 37; 
const int A_SPEED = 41;

const int B_DIRECTION = 31;
const int B_BRAKE = 39;
const int B_SPEED = 35;

const int pingPin = 5;

unsigned int duration, inches, distance;

//const int sdPin = 4;
const int ssWifi = 10;

int LeftRotation = 500; //900;
int RightRotation = 500; //1000;

int wallDistance = 10;

char ssid[] = "ssid";
char pass[] = "password";
int status = WL_IDLE_STATUS; 

WiFiServer server(80);

void setup() {
  
  pinMode(10,OUTPUT); // pin 10 must be left as an OUTPUT so it can be set HIGH
  digitalWrite(10,HIGH); // pin 10 needs to be put HIGH to explicitly deselect the WiFi HDG104 chip[code]
  pinMode(53, OUTPUT);

  Serial.begin(9600);
  
   // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present"); 
    while(true);        // don't continue
  } 

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) { 
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(ssid);                   // print the network name (SSID);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:    
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  } 
  server.begin();                           // start the web server on port 80
  printWifiStatus();                        // you're connected now, so print out the status
  
//  while(!Serial) ;
//
//  // attempt to connect using WEP encryption:
//  Serial.println("Attempting to connect to locadotnet2...");
//  status = WiFi.begin(ssid, pass);
//
//  // if you're not connected, stop here:
//  if ( status != WL_CONNECTED) {
//    Serial.println("Couldn't get a wifi connection");
//    while(true);
//  }
//  // if you are connected, print out info about the connection:
//  else {
//    Serial.print("Connected to locadotnet2!  Your IP is: ");
//    Serial.println(WiFi.localIP());
//    //server.begin();
//    printWifiStatus();
//    
//  }

  //Setup Channel A
  pinMode(A_DIRECTION, OUTPUT); //Initiates Motor Channel A pin
  pinMode(A_BRAKE, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(B_DIRECTION, OUTPUT); //Initiates Motor Channel A pin
  pinMode(B_BRAKE, OUTPUT);  //Initiates Brake Channel A pin

  delay(2000);

}

void loop()
{
    WiFiClient client = server.available();   // listen for incoming clients
    if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {  
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:    
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> turn the LED on pin 9 on<br>");
            client.print("Click <a href=\"/L\">here</a> turn the LED on pin 9 off<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;         
          } 
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }     
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(9, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(9, LOW);                // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
  
  /*
  //listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println("Refresh: 5");  // refresh the page automatically every 5 sec
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");
          // output the value of each analog input pin
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
            int sensorReading = analogRead(analogChannel);
            client.print("analog input ");
            client.print(analogChannel);
            client.print(" is ");
            client.print(sensorReading);
            client.println("<br />");       
          }
          client.println("</html>");
           break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    
    // close the connection:
    client.stop();
    Serial.println("client disonnected");
  }
  ////////
  */
  int distance = PingRead();
  
  //for debugging 
  Serial.println(distance);
  //MotorConsumption();
  
  if (distance > wallDistance)
  {
    //go forward
    forward();
  }
  else
  {
    //stop driving
    engageBrake();
    
    delay(2000);
    
    
    //disengageBrake();
    //right();
      if (distance < wallDistance)
      {
        disengageBrake();
        
        right();
        delay(RightRotation);
        
        engageBrake();
        
        delay(100);
        disengageBrake();
        forward();
      }
  }
  
  //delay(1000);
}



/////////////////////????////
//  drive motor functions  //
/////////////////////////////


void engageBrake() {
  digitalWrite(B_BRAKE, HIGH);  //Engage the Brake for Channel A
  digitalWrite(A_BRAKE, HIGH);  //Engage the Brake for Channel B
}

void disengageBrake() {
  digitalWrite(B_BRAKE, LOW);  //Engage the Brake for Channel A
  digitalWrite(A_BRAKE, LOW);  //Engage the Brake for Channel B
}


void forward() {
 
  //Motor A forward @ full speed
  digitalWrite(A_DIRECTION, LOW); //Establishes forward direction of Channel A
  digitalWrite(A_BRAKE, LOW);   //Disengage the Brake for Channel A
  analogWrite(A_SPEED, 255);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(B_DIRECTION, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(B_BRAKE, LOW);   //Disengage the Brake for Channel B
  analogWrite(B_SPEED, 255);    //Spins the motor on Channel B at half speed
 
}

void backward() {
 
  //Motor A forward @ full speed
  digitalWrite(A_DIRECTION, HIGH); //Establishes forward direction of Channel A
  digitalWrite(A_BRAKE, LOW);   //Disengage the Brake for Channel A
  analogWrite(A_SPEED, 255);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(B_DIRECTION, LOW);  //Establishes backward direction of Channel B
  digitalWrite(B_BRAKE, LOW);   //Disengage the Brake for Channel B
  analogWrite(B_SPEED, 255);    //Spins the motor on Channel B at half speed
 
}

void left() {
 
  //Motor A forward @ full speed
  digitalWrite(A_DIRECTION, HIGH); //Establishes forward direction of Channel A
  digitalWrite(A_BRAKE, LOW);   //Disengage the Brake for Channel A
  analogWrite(A_SPEED, 255);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(B_DIRECTION, HIGH);  //Establishes backward direction of Channel B
  digitalWrite(B_BRAKE, LOW);   //Disengage the Brake for Channel B
  analogWrite(B_SPEED, 255);    //Spins the motor on Channel B at half speed
 
}

void right() {
 
  //Motor A forward @ full speed
  digitalWrite(A_DIRECTION, LOW); //Establishes forward direction of Channel A
  digitalWrite(A_BRAKE, LOW);   //Disengage the Brake for Channel A
  analogWrite(A_SPEED, 255);   //Spins the motor on Channel A at full speed

  //Motor B backward @ half speed
  digitalWrite(B_DIRECTION, LOW);  //Establishes backward direction of Channel B
  digitalWrite(B_BRAKE, LOW);   //Disengage the Brake for Channel B
  analogWrite(B_SPEED, 255);    //Spins the motor on Channel B at half speed
 
}


/////////////////////////////////
//  distance sensor functions  //
/////////////////////////////////

long PingRead() {


  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(duration);
  
  return inches;
  //delay(100);
 
}

//for PING second to inch conversion

long microsecondsToInches(long microseconds)
{
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

///////////////////////////////////////
//     Motor Consumption Function    //
///////////////////////////////////////

void MotorConsumption() {
  Serial.print("sensor 0: ");
  Serial.println(analogRead(SNS0));
  Serial.print("sensor 1: ");
  Serial.println(analogRead(SNS1));
  Serial.println("**************");
}

/*
  
void compareDistance()
{
  if (leftDistance>rightDistance) //if left is less obstructed 
  {
    leftMotor.write(LBackward); 
    rightMotor.write(RForward); //turn left
    delay(500); 
  }
  else if (rightDistance>leftDistance) //if right is less obstructed
  {
    leftMotor.write(LForward);
    rightMotor.write(RBackward); //turn right
    delay(500);
  }
   else //if they are equally obstructed
  {
    leftMotor.write(LForward); 
    rightMotor.write(RBackward); //turn 180 degrees
    delay(1000);
  }
}

*/


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
