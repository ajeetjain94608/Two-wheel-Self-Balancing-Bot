#include <SPI.h>
#include <RF24.h>

// Define the radio
RF24 radio(10, 9);

// Define constants
const byte add_1 = 5;
const byte add_2 = 10;
const int VRx = A6; // Analog pin for x-axis 
const int VRy = A7; // Analog pin for y-axis
const int SW  = 8;  // Digital pin for the button
const int button1 = 6;
const int button2 = 5;

// Define variables
int x_value, y_value; 

// Define struct
typedef struct struct_message {
 int front=0;
 int back=0;
 int left=0;
 int right=0;
 int sw1=0;
 int sw2=0;
 int sw3=0;
} struct_message;

// Create object of struct_message
struct_message data;

void setup() {
   Serial.begin(9600);
   int result = radio.begin();
   
   // Set pin modes
   pinMode(SW, INPUT_PULLUP);
   pinMode(button1, INPUT_PULLUP);
   pinMode(button2, INPUT_PULLUP);

   // Check if radio is working
   if (result) {
     Serial.println("NRF module Working");
   } else {
     Serial.println("NRF module ERROR ");
   }
   
   // Set up radio
   radio.openWritingPipe(add_1);
   radio.openReadingPipe(1, add_2);
   radio.setDataRate(RF24_2MBPS); 
   radio.setPALevel(RF24_PA_MIN);
   radio.stopListening();
}

void loop() {
   // Read analog values
   x_value = analogRead(VRx);
   y_value = analogRead(VRy);
   
   // Check x_value and set data
  if (x_value > 900) {
    data.left = 1;
    data.right = 0;
   } else if (x_value < 200) {
     data.left = 0;
     data.right = 1;
   
   } else {
     data.left = 0;
     data.right = 0;
   } 


   // Check y_value and set data
   if (y_value > 900) {
     data.front = 0;
     data.back = 1;
   } else if (y_value < 200) {
     data.front = 1;
     data.back = 0;
   } else {
     data.back = 0;
     data.front = 0;
   }
   
   // Read digital values and set data
   data.sw1 = !digitalRead(SW);
   data.sw2 = !digitalRead(button1);
   data.sw3 = !digitalRead(button2);
   // Write data to radio
   radio.write( (uint8_t *) &data, sizeof(struct_message));
}
