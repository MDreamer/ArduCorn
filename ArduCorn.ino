#include "FastLED.h"
#define NUM_LEDS 80

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

#define MAX_LEVEL 10 //how many levels does the spacecorn got
#define MAX_RING 8 //how many led does the each ring got

CRGB leds[NUM_LEDS];

const int flash_const = 25;

byte corn_mapping[MAX_LEVEL][MAX_RING];

bool debug_mode = true;

struct kernel_corn
{ 
  bool is_kernel_popped;
  long last_pop_ms;
};

kernel_corn popped_kernels[NUM_LEDS];
// Array for holding the UV_LED lights behaviours and pins
// PinNo, Value, periodePulse, periodePulse per LED
int UV_arr_pin[UV_LEDS][4];



int max_led = 0;
int min_led = 0;
int light_color;

int time_between_lights;

int CRGB_color;
 
// Pin 13 has an LED connected on most Arduino boards. for indication/debugging
int led = 13;

long time=0;

bool onshot_arm = true;

long last_check;

//redout - useed to indicate error
void redout()
{
  for (int i=0; i < NUM_LEDS; i++)
    leds[i] = CRGB::Red; 
}
        
// test indication - when the corn is booted it gives a visual indication
// of the LED driver by painting 3 rings in RGB
void reset_corn()
{
  for (int i=0; i < NUM_LEDS; i++)
        leds[i] = CRGB::Black; 
        
  for (int i=0; i < MAX_RING; i++)
    leds[i] = CRGB::Red;

  for (int i=(MAX_RING*1); i < (MAX_RING*1) + MAX_RING; i++)
    leds[i] = CRGB::Red;

  for (int i=(MAX_RING*5); i < (MAX_RING*5) + MAX_RING; i++)
    leds[i] = CRGB::Green;
  for (int i=(MAX_RING*6); i < (MAX_RING*6) + MAX_RING; i++)
    leds[i] = CRGB::Green;

  for (int i=(MAX_RING*8); i < (MAX_RING*8) + MAX_RING; i++)
    leds[i] = CRGB::Blue;
  for (int i=(MAX_RING*9); i < (MAX_RING*9) + MAX_RING; i++)
    leds[i] = CRGB::Blue;
    
  FastLED.show(); 
}
// the setup routine runs once when you press reset:
void setup() {
  UV_arr_pin[0][0] = LED_PIN0;
  UV_arr_pin[1][0] = LED_PIN1;
  UV_arr_pin[2][0] = LED_PIN2;
  UV_arr_pin[3][0] = LED_PIN3;
  UV_arr_pin[4][0] = LED_PIN4;
  UV_arr_pin[5][0] = LED_PIN5;
  
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);

  //init all the LED pins as outputs
  for (int i=0;i<UV_LEDS;i++)
  {
    pinMode(UV_arr_pin[i][0], OUTPUT);
    UV_arr_pin[i][2] = random(3000,6000);
    UV_arr_pin[i][3] = random(1000,3000);
  }
  //analogWrite(UV_arr[0],200); /// jut a test for the PWM
  // initialize serial:
  Serial.begin(9600);
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  Serial.setTimeout(10);
  randomSeed(analogRead(0));
  FastLED.addLeds<WS2811, 2>(leds, NUM_LEDS); 
  //clear corn
  for (int i=0; i <= NUM_LEDS; i++)
    leds[i] = CRGB::Black; 

  reset_corn();
  
   
   
}

// the loop routine runs over and over again forever:
void loop() 
{
  // TOOD: random every X period of time - change periodePulse & displacePulse
  //       for every UL_LED in the array. to it per led so everything won't change
  //       at once



  time = millis();
  
  for (int i=0;i < UV_LEDS; i++)
  {
    UV_arr_pin[i][1] = 128+127*cos(2*PI/UV_arr_pin[i][2]*(UV_arr_pin[i][3]-time));
    analogWrite(UV_arr_pin[i][0],UV_arr_pin[i][1]); /// jut a test for the PWM
  }
  /*
  Serial.print("0 ");
  Serial.println(UV_arr_pin[0][0]);
  Serial.print("1 ");
  Serial.println(UV_arr_pin[0][1]);
  Serial.print("2 ");
  Serial.println(UV_arr_pin[0][2]);
  Serial.print("3 ");
  Serial.println(UV_arr_pin[0][3]);
  Serial.println("");
  Serial.println("");
  delay(1000);
  */
  
  if (Serial.available() > 0) 
  {
    // onshot to kill all the led after comm line is working
    if (onshot_arm == true)
    {
      for (int i=0; i <= NUM_LEDS; i++)
        leds[i] = CRGB::Black; 
        onshot_arm = false;
    }
    
    byte payload[MSG_SIZE];
    Serial.readBytes(payload,MSG_SIZE);
    if (debug_mode)
      for (int i = 0; i < MSG_SIZE; i++)
      {
        Serial.write(payload[i]);
        Serial.print(" ");
      }
    //Serial.println(payload[1]);
    //Serial.write(payload,MSG_SIZE);
    //FastLED.show();
    //TODO: fix the checksum. Something wrong with the end byte checkinh
    if (checksum(payload)) //check the checksum + start&end byte
    {  
      if (debug_mode)
        Serial.println("checksum OK");
      if (payload[1] == 50) //it means the a led command is on
      {
        if (debug_mode)
          Serial.println("LED command detected");
        //tricky part - prepair for a headache... 
        // conversion form "octal" sensor data mapping to the led chain
        // for exmaple 22 in sensors' input is device 1
        // (actually 3 coz we count form 0) and channel 3 (actually 3 coz we count form 0)
        // so it will be 22 in sensors = (2(first digit) * 8) + 2 (second digit) = 18 in LED
        
        
        int led_chain;
        led_chain = ((payload[2]/10)*8)+(payload[2]%10);
        popped_kernels[led_chain].is_kernel_popped = true;
        //SPI dev = first digit
        //ADC channel = second digit

        if (debug_mode)
        {
          Serial.print("got num ");
          Serial.println(payload[2]);
          
          Serial.print("popping led number ");
          Serial.println(led_chain);

          Serial.print("1 digit");
          Serial.println((payload[2]/10)*8);

          Serial.print("2 digit");
          Serial.println(payload[2]%8);
        }
        
        int sensor_loc = payload[2];
        
        
        kernel_pop(led_chain);

        //just for testing
        //single_hit_colowipe(led_chain);
        popped_kernels[led_chain].last_pop_ms = millis();
      }
      else if (payload[1] == 80) // reset colors
      {
        reset_corn();
      }
    }
    else //checksum error
    {
      redout(); // error indication
      //in case something went wornd the one byte is corrupted is "flashes" the buffer
      while (Serial.available() > 0)
        Serial.read();
    }    
  }
  
  last_check = millis();
  
  //This part is doing thet flashign LED after a hit
  for (int j = 0; j < NUM_LEDS;j++)
  {
    if (popped_kernels[j].is_kernel_popped == true)
    {
      //when hit flash-on-off for 500ms the kernel that got hit
      if ((abs(last_check - popped_kernels[j].last_pop_ms) > 0) and 
          (abs(last_check - popped_kernels[j].last_pop_ms) < flash_const*1))
      {
        leds[j] = CRGB::White;
      }
      else if ((abs(last_check - popped_kernels[j].last_pop_ms) > (flash_const*1)) and 
                abs((last_check - popped_kernels[j].last_pop_ms) < (flash_const*2)))
      {
        leds[j] = CRGB::Black;
      }
      else if (abs((last_check - popped_kernels[j].last_pop_ms) > (flash_const*2)) and 
               abs((last_check - popped_kernels[j].last_pop_ms) < (flash_const*3)))
      {
        leds[j] = CRGB::White;
      }
      else if (abs((last_check - popped_kernels[j].last_pop_ms) > (flash_const*3)) and 
               abs((last_check - popped_kernels[j].last_pop_ms) < (flash_const*4)))
      {
        leds[j] = CRGB::Black;
      }
      else if (abs((last_check - popped_kernels[j].last_pop_ms) > (flash_const*4)) and 
               abs((last_check - popped_kernels[j].last_pop_ms) < (flash_const*5)))
      {
        leds[j] = CRGB::White;
        popped_kernels[j].is_kernel_popped = false;
      }
    }
  }
  

  FastLED.show();
  
}

//turns on the kernel that got
void single_hit_colowipe(int hit_kernel, int sensor_loc)
{
  for (int i; i < NUM_LEDS; i++)
    leds[i] = CRGB::Black;

  leds[hit_kernel] = CRGB::White;
    
  FastLED.show();
}
//this functio calculated on the corn's cylinder the kernels that need to pop next to the one that got it
// TODO: needs to be timed hit retina for the main kernel - turns white
void kernel_pop(int hit_kernel)
{
  /*
      |          
   --+-+--
     |X|
   --+-+--
      |
     */
  leds[hit_kernel].red   = 255;
  leds[hit_kernel].green = 255;
  leds[hit_kernel].blue  = 255;

  /*
     X|X          
   --+-+--
     | |
   --+-+--
      |  
     */
     //THe +1 is because the levels are not aligned and in one bottle indentation
  /*   
  if ((hit_kernel+(MAX_RING*2)-1) < NUM_LEDS)
  {
    leds[hit_kernel+(MAX_RING*2)-1].red = random(0,255);
    leds[hit_kernel+(MAX_RING*2)-1].green =  random(0,255);
    leds[hit_kernel+(MAX_RING*2)-1].blue  =  random(0,255);  
  }
  */
  if ((hit_kernel+MAX_RING) < NUM_LEDS)
  {
    leds[hit_kernel+MAX_RING].red = random(0,255);
    leds[hit_kernel+MAX_RING].green =  random(0,255);
    leds[hit_kernel+MAX_RING].blue  =  random(0,255);  
  }
  /*
      |          
   --+-+--
    X| |
   --+-+--
      |
     */
  if ((hit_kernel-1 > 0) and !(hit_kernel % 10 == 0))
  {
    leds[hit_kernel-1].red = random(0,255);
    leds[hit_kernel-1].green =  random(0,255);
    leds[hit_kernel-1].blue  =  random(0,255);  
  }
  else if (hit_kernel == 0)
  {
    //if kernel 0 was hit 
    leds[MAX_RING-1].red = random(0,255);
    leds[MAX_RING-1].green =  random(0,255);
    leds[MAX_RING-1].blue  =  random(0,255);  
  } else if (hit_kernel % 10 == 0) // if kernel with 0 single digit 0 was hit - light led 7 on the the same ring 
    {
      leds[hit_kernel+MAX_RING-1].red = random(0,255);
      leds[hit_kernel+MAX_RING-1].green =  random(0,255);
      leds[hit_kernel+MAX_RING-1].blue  =  random(0,255);  
    }
  /*
      |          
   --+-+--
     | |X
   --+-+--
      |
     */
  if ((hit_kernel+1 < NUM_LEDS) and !(hit_kernel % 10 == 7))
  {
    leds[hit_kernel+1].red = random(0,255);
    leds[hit_kernel+1].green =  random(0,255);
    leds[hit_kernel+1].blue  =  random(0,255);  
  }
  else if (hit_kernel == NUM_LEDS-1)
  {
    leds[NUM_LEDS-MAX_RING+1].red = random(0,255);
    leds[NUM_LEDS-MAX_RING+1].green =  random(0,255);
    leds[NUM_LEDS-MAX_RING+1].blue  =  random(0,255);    
  }else if (hit_kernel % 10 == 7) // if led 7 was hit - light led 0 on the the same ring 
    {
      leds[hit_kernel-MAX_RING+1].red = random(0,255);
      leds[hit_kernel-MAX_RING+1].green =  random(0,255);
      leds[hit_kernel-MAX_RING+1].blue  =  random(0,255);  
    }
  /*
      |          
   --+-+--
     | |
   --+-+--
     X|X
     */
    
  if (hit_kernel-MAX_RING > 0)
  {
    leds[hit_kernel-MAX_RING].red = random(0,255);
    leds[hit_kernel-MAX_RING].green =  random(0,255);
    leds[hit_kernel-MAX_RING].blue  =  random(0,255);  
  }
   /*
  if (hit_kernel-(MAX_RING*2)-1 < 0)
  {
    leds[hit_kernel-(MAX_RING*2)-1].red = random(0,255);
    leds[hit_kernel-(MAX_RING*2)-1].green =  random(0,255);
    leds[hit_kernel-(MAX_RING*2)-1].blue  =  random(0,255);  
  }
  */
}


// check sum data check up 
bool checksum(byte payload_check[])
{
  byte tempChkSum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) // don't calc the checksum
  {
    tempChkSum += payload_check[i];
  }
   
  if ((tempChkSum == payload_check[MSG_SIZE - 1]) and 
      (payload_check[0] == 115) and //'s' char = 0x73
      (payload_check[7] == 114)) //'r' char  = 0x72
      {
        if (debug_mode)
            Serial.println("Passed checksum + prefix + suffix");
        return true;
      }
  else
    return false; 
}

