#include <Wire.h>

int startProcess = 4;                   // to trigger the measurement in sensor chaining approach
int restartUltrasoundAcq;
char command;

unsigned int distB, distR, distC, distL;

unsigned int voltages[9];               //int on 2 bytes | long on 4 bytes

union I2C_data {
   unsigned int  dataInt  [4];
   unsigned char dataChar [8];
};

I2C_data dataToSend;

void setup() 
{
  restartUltrasoundAcq = 0;
  
  // put your setup code here, to run once:
  delay (400);          // 400 ms

  pinMode      (startProcess, OUTPUT);
  digitalWrite (startProcess, LOW);

  //init I2C
  Wire.begin(0x3d);                     // join i2c bus with address 0x3d
  Wire.onReceive (receiveEvent);        // interrupt handler for incoming messages
  Wire.onRequest (requestEvent);        // interrupt handler for when data is wanted

  //start aquisition process
  delay (5);                            // 5 ms
  digitalWrite (startProcess, HIGH);    // pull RX pin high for the first sensor
  delay (60);                           // for at least 200 us up to 96 ms  
  pinMode      (startProcess, INPUT);   // return pin state to high impedance

  //init serial
  Serial.begin(115200);
}

void loop() 
{
  if (restartUltrasoundAcq == 1)
      {
      //start aquisition process
      delay (5);                            // 5 ms
      digitalWrite (startProcess, HIGH);    // pull RX pin high for the first sensor
      delay (60);                           // for at least 200 us up to 96 ms  
      pinMode      (startProcess, INPUT);   // return pin state to high impedance      
    
      restartUltrasoundAcq = 0;
      }
  
  //get LEFT data sensor
  for (int i = 0; i < 9; i++)
    {
    voltages[i] = analogRead (A0);  
    delay (10);
    }

  quickSort(voltages, 0, 8);
  distL = voltages[4];

  //get CENTER data sensor
  for (int i = 0; i < 9; i++)
    {
    voltages[i] = analogRead (A1);  
    delay (10);
    }

  quickSort(voltages, 0, 8);
  distC = voltages[4];

  //get RIGHT data sensor
  for (int i = 0; i < 9; i++)
    {
    voltages[i] = analogRead (A2);  
    delay (10);
    }

  quickSort(voltages, 0, 8);
  distR = voltages[4];

  //get BACK data sensor
  for (int i = 0; i < 9; i++)
    {
    voltages[i] = analogRead (A3);  
    delay (10);
    }

  quickSort(voltages, 0, 8);
  distB = voltages[4];

  noInterrupts();                   //   If interrupts are disabled and one or more 
  dataToSend.dataInt[0] = distL;    // interrupts event occurs (e.g. an I2C request)    
  dataToSend.dataInt[1] = distC;    // then that interrupt or interrupts will be 
  dataToSend.dataInt[2] = distR;    // handled when they are enabled again, in priority 
  dataToSend.dataInt[3] = distB;    // order.
  interrupts();

  //Serial.print ("SL= ");  Serial.print (distL); Serial.print (" SC= "); Serial.print (distC);
  //Serial.print (" SR= "); Serial.print (distR); Serial.print (" SB= "); Serial.println (distB); 
}



/*
 *   I2C CODE !!!!!
 */

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
     case 0x55: 
        Wire.write(dataToSend.dataChar, 8); 
        break;
     case 0x56:
        Wire.write("Restart!"); 
     
        restartUltrasoundAcq = 1;
        break;
     }  // end of switch

  Serial.print ("I2C device received a data request [2] !!!\n");
}

//incomming messages
void receiveEvent (int howMany)
{
  command = Wire.read ();  // remember command for when we get request
  
  Serial.print ("I2C device received a request [1]: "); Serial.println (command);
} // end of receiveEvent


/*
 *   Only Quicksort CODE !!!!!
 */

// A utility function to swap two elements 
void swap(unsigned int* a, unsigned int* b) 
{ 
    unsigned int t = *a; 
    *a = *b; 
    *b = t; 
} 
  
/* This function takes last element as pivot, places 
   the pivot element at its correct position in sorted 
    array, and places all smaller (smaller than pivot) 
   to left of pivot and all greater elements to right 
   of pivot */
int partition (unsigned int arr[], int low, int high) 
{ 
    unsigned int pivot = arr[high];    // pivot 
    int i = (low - 1);  // Index of smaller element 
  
    for (int j = low; j <= high- 1; j++) 
    { 
        // If current element is smaller than the pivot 
        if (arr[j] < pivot) 
        { 
            i++;    // increment index of smaller element 
            swap(&arr[i], &arr[j]); 
        } 
    } 
    swap(&arr[i + 1], &arr[high]); 
    return (i + 1); 
} 
  
/* The main function that implements QuickSort 
 arr[] --> Array to be sorted, 
  low  --> Starting index, 
  high  --> Ending index */
void quickSort(unsigned int arr[], int low, int high) 
{ 
    if (low < high) 
    { 
        /* pi is partitioning index, arr[p] is now 
           at right place */
        int pi = partition(arr, low, high); 
  
        // Separately sort elements before 
        // partition and after partition 
        quickSort(arr, low, pi - 1); 
        quickSort(arr, pi + 1, high); 
    } 
} 
