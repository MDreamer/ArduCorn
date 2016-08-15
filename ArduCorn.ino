#include "FastLED.h"
#define NUM_LEDS 50

//MAX NUM of UV LEDs
#define UV_LEDS 6

//List of the pins that are connected to the NPNs
//Those pins are also have PWM
#define LED_PIN0 3
#define LED_PIN1 5
#define LED_PIN2 6
#define LED_PIN3 9
#define LED_PIN4 10
#define LED_PIN5 11

//byte 0 - start byte
//byte 1 - command - addressble LED/UV light
//byte 2 - data - No# of LED/ UV LED
//byte 3 - data (TBD) for future use if we'll have time? contol colours?
//byte 4 - data (TBD)
//byte 5 - data (TBD)
//byte 6 - running num seq
//byte 7 - end byte
//byte 8 - checksum
#define MSG_SIZE 9

CRGB leds[NUM_LEDS];

int UV_arr[UV_LEDS] = {LED_PIN0,LED_PIN1,LED_PIN2,LED_PIN3,LED_PIN4,LED_PIN5};

int max_led = 0;
int min_led = 0;
int light_color;

int time_between_lights;

int CRGB_color;
 
// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

// fade-dimm configuration
int periodePulse = random(3000,6000);
int displacePulse = random(1000,3000);     
long time=0;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);

  //init all the LED pins as outputs
  for (int i=0;i<UV_LEDS;i++)
    pinMode(UV_arr[i], OUTPUT);
  //analogWrite(UV_arr[0],200); /// jut a test for the PWM
  // initialize serial:
  Serial.begin(9600);
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  Serial.setTimeout(10);
  randomSeed(analogRead(0));
  FastLED.addLeds<NEOPIXEL, 2>(leds, NUM_LEDS); 
  //clear cloud
  for (int i=0; i <= NUM_LEDS; i++)
    leds[i] = CRGB::Black; 
   FastLED.show(); 
}

// the loop routine runs over and over again forever:
void loop() 
{
  // TOOD: random every X period of time - change periodePulse & displacePulse
  //       for every UL_LED in the array. to it per led so everything won't change
  //       at once
  time = millis();
  byte color_value = 128+127*cos(2*PI/periodePulse*(displacePulse-time));
  analogWrite(UV_arr[0],color_value); /// jut a test for the PWM
  
  if (Serial.available() > 0) 
  {
    byte payload[MSG_SIZE];
    Serial.readBytes(payload,MSG_SIZE);
    
    //FastLED.show();
    //TODO: fix the checksum. Something wrong with the end byte checkinh
    //if (checksum(payload)) //check the checksum + start&end byte
    //{  
      //
      if (payload[1] == 50) //it means the a led command is on
      {
        
        for (int i; i< NUM_LEDS; i++)
          leds[i] = CRGB::Black;
      
        leds[payload[2]] = CRGB::Blue;
  
        FastLED.show();
      }
    //}
    //else //in case something went wornd the one byte is corrupted is "flashes" the buffer
    //  while (Serial.available() > 0)
    //    Serial.read();
    
  }
  
  
}
// check sum data check up 
bool checksum(byte payload_check[])
{
  byte tempChkSum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) // don't calc the checksum
  {
    tempChkSum += payload_check[i];
  }
  if (payload_check[7] == 114)
    digitalWrite(led, HIGH);
  if (tempChkSum == payload_check[MSG_SIZE - 1] and 
      payload_check[0] == 115 and //'s' char 
      payload_check[7] == 114 ) //'r' char 
    return true;
  else
    return false; 
}

