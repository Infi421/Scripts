#include <ros.h>
#include <std_msgs/String.h>
#include <FastLED.h>

#define NUM_LEDS 48
#define DATA_PIN 3
#define BRIGHTNESS 64
#define LED_TYPE WS2811
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

ros::NodeHandle nh;

String mode = "";

void receiveMode(const std_msgs::String& msg) {
  mode = msg.data;
  //Serial.println(mode);
}

ros::Subscriber<std_msgs::String> sub("operation_mode", receiveMode);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(9600);
  //Serial.println("Serial initialized");
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip).setDither(BRIGHTNESS < 255);
  FastLED.setBrightness(BRIGHTNESS);
}

void red()
{
  for (int i = 0; i< NUM_LEDS; i+=4)
  {
    for (int j = i; j<i+4; j+=1)
    {
      leds[j] = CRGB::Red;
    }
    FastLED.show();
    delay(100);
  }
}

void green()
{
  for (int i = 0; i< NUM_LEDS; i+=4)
  {
    for (int j = i; j<i+4; j+=1)
    {
      leds[j] = CRGB::Green;
    }
    FastLED.show();
    delay(100);
  }
}

void yellow()
{
  for (int i = 0; i< NUM_LEDS; i+=4)
  {
    for (int j = i; j<i+4; j+=1)
    {
      leds[j] = CRGB::Yellow;
    }
    FastLED.show();
    delay(100);
  }
}

void loop() {
 
  nh.spinOnce();

  if(mode.compareTo("auto") == 0)
  {
    //Serial.println("Autonomous Detected!!!");
    red();
  }
  else if(mode.compareTo("manual") == 0)
  {
    green();
  }
  else
  {
    yellow();
  }   
}
