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

byte corn_mapping[MAX_LEVEL][MAX_RING];

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
  //clear cloud
  for (int i=0; i <= NUM_LEDS; i++)
    leds[i] = CRGB::Black; 

  //test indication
  leds[10].red   = 255;
  leds[10].green = 0;
  leds[10].blue  = 0;
  
  leds[21].red   = 0;
  leds[21].green = 255;
  leds[21].blue  = 0;

  leds[32].red   = 0;
  leds[32].green = 0;
  leds[32].blue  = 255;
   FastLED.show(); 
   
}

// the loop routine runs over and over again forever:
void loop() 
{
  // TOOD: random every X period of time - change periodePulse & displacePulse
  //       for every UL_LED in the array. to it per led so everything won't change
  //       at once



  time = millis();
  
  for (int i;i < UV_LEDS; i++)
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
    //Serial.write(payload,MSG_SIZE);
    //FastLED.show();
    //TODO: fix the checksum. Something wrong with the end byte checkinh
    //if (checksum(payload)) //check the checksum + start&end byte
    //{  
      //
      if (payload[1] == 50) //it means the a led command is on
      {
        //tricky part - prepair for a headache... 
        // conversion form "octal" sensor data mapping to the led chain
        // for exmaple 22 in sensors' input is device 1
        // (actually 3 coz we count form 0) and channel 3 (actually 3 coz we count form 0)
        // so it will be 22 in sensors = (2(first digit) * 8) + 2 (second digit) = 18 in LED
       
        int led_chain;
        if (payload[2] >= 10)
           led_chain = ((payload[2]/10)*8)+(payload[2]%8);
        else
           led_chain = (payload[2]%8);

        //SPI dev = first digit
        //ADC channel = second digit
        
        int sensor_loc = (payload[2]/10) +(payload[2]%8);
        Serial.println(sensor_loc);
        
        kernel_pop(led_chain,sensor_loc);
      }
    //}
    //else //in case something went wornd the one byte is corrupted is "flashes" the buffer
    //  while (Serial.available() > 0)
    //    Serial.read();
    
  }
  //leds[3] = CRGB::Blue;


  FastLED.show();
  
}

//this functio calculated on the corn's cylinder the kernels that need to pop next to the one that got it
// TODO: needs to be timed hit retina for the main kernel - turns white
void kernel_pop(int hit_kernel,int octRep)
{
  /*
     | |          
   --+-+--
     |X|
   --+-+--
     | |
     */
  leds[hit_kernel].red   = 255;
  leds[hit_kernel].green = 255;
  leds[hit_kernel].blue  = 255;

  /*
     |X|          
   --+-+--
     | |
   --+-+--
     | |  
     */
  if (hit_kernel-MAX_RING >= 0)
  {
    leds[hit_kernel-MAX_RING].red = random(0,255);
    leds[hit_kernel-MAX_RING].green =  random(0,255);
    leds[hit_kernel-MAX_RING].blue  =  random(0,255);  
  }
  /*
     | |          
   --+-+--
    X| |
   --+-+--
     | |
     */
  if (hit_kernel-1 >= 0)
  {
    leds[hit_kernel-1].red = random(0,255);
    leds[hit_kernel-1].green =  random(0,255);
    leds[hit_kernel-1].blue  =  random(0,255);  
  }
  /*
     | |          
   --+-+--
     | |X
   --+-+--
     | |
     */
  if (hit_kernel+1 < NUM_LEDS-1)
  {
    leds[hit_kernel+1].red = random(0,255);
    leds[hit_kernel+1].green =  random(0,255);
    leds[hit_kernel+1].blue  =  random(0,255);  
  }
  /*
     | |          
   --+-+--
     | |
   --+-+--
     |X|
     */
  if (hit_kernel+MAX_RING < NUM_LEDS)
  {
    leds[hit_kernel+MAX_RING].red = random(0,255);
    leds[hit_kernel+MAX_RING].green =  random(0,255);
    leds[hit_kernel+MAX_RING].blue  =  random(0,255);  
  }
}

int decimal_octal(int n) /* Function to convert decimal to octal */
{
    int rem, i=1, octal=0;
    while (n!=0)
    {
        rem=n%8;
        n/=8;
        octal+=rem*i;
        i*=10;
    }
    return octal;
}

int octal_decimal(int n) /* Function to convert octal to decimal */
{
    int decimal=0, i=0, rem;
    while (n!=0)
    {
        rem = n%10;
        n/=10;
        decimal += rem*pow(8,i);
        ++i;
    }
    return decimal;
}

// check sum data check up 
bool checksum(byte payload_check[])
{
  byte tempChkSum = 0;
  for (int i = 0; i < MSG_SIZE - 1; i++) // don't calc the checksum
  {
    tempChkSum += payload_check[i];
  }
  
  if (tempChkSum == payload_check[MSG_SIZE - 1] and 
      payload_check[0] == 0x73 and //'s' char 
      payload_check[7] == 0x72) //'r' char 
    return true;
  else
    return false; 
}

