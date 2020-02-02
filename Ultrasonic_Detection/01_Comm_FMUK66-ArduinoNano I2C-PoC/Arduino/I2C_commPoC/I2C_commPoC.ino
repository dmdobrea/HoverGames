#include <Wire.h>

char command;

void setup() 
{
  //init I2C
  Wire.begin(0x3d);                 // join i2c bus with address 0x3d
  Wire.onReceive (receiveEvent);    // interrupt handler for incoming messages
  Wire.onRequest (requestEvent);    // interrupt handler for when data is wanted

  //init serial
  Serial.begin(9600);  // start Serial for output 
  Serial.print ("The UART was init !!!\n");
}

void loop() {
  // put your main code here, to run repeatedly:

  // all done by interrupts
}

//on the Drone the I2C transfer function is:  "if ( OK != transfer ( &cmd, 1, data, 3) )"
// 1. the first part (transfer ( &cmd, 1, ..., ....)) will write to the Nano Arduino different value as 0x55, 0x56, 0x57
//    and will triger "receiveEvent (int howMany)" function from Arduino Nano
// 2. the second part (transfer ( ..., ..., data, 3)) will request from the slave (Nano Arduino) three values (unsigned char)
//    and will triger "requestEvent()" function from Arduino Nano 

//data request from master
void requestEvent()
{
  switch (command)
     {
     case 0x55: Wire.write("Hi!"); break;
     case 0x56: Wire.write("Oo!"); break;
     case 0x57: Wire.write("By!"); break;
     }  // end of switch

  Serial.print ("I2C device received a data request [2] !!!\n");
}

//incomming messages
void receiveEvent (int howMany)
{
  command = Wire.read ();  // remember command for when we get request
  
  Serial.print ("I2C device received a data [1] !!!\n");
} // end of receiveEvent
