/* 
 * Button Example for Rosserial
 */

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

#include "DHT.h"

#define DHTPIN 7     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
DHT dht(DHTPIN, DHTTYPE);

ros::NodeHandle nh;

std_msgs::Bool pushed_msg;
ros::Publisher pub_button("pushed", &pushed_msg);

std_msgs::Float32 temperature_msg;
ros::Publisher pub_temperature("temperature", &temperature_msg);
std_msgs::Float32 humidity_msg;
ros::Publisher pub_humidity("humidity", &humidity_msg);
std_msgs::Float32 pressure_msg;
ros::Publisher pub_pressure("pressure", &pressure_msg);

const int button_pin1 = 2;
const int button_pin2 = 3;
const int led_pin = 13;

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;
volatile unsigned long last_time_sensing = 0;

void setup()
{
  bmp.begin();
  dht.begin();

  nh.initNode();
  nh.advertise(pub_button);
  nh.advertise(pub_temperature);
  nh.advertise(pub_humidity);
  nh.advertise(pub_pressure);
  
  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin1, INPUT);
  pinMode(button_pin2, INPUT);
  
  //Enable the pullup resistor on the button
  //digitalWrite(button_pin, HIGH);
  
  //The button is a normally button
  last_reading = digitalRead(button_pin1) | digitalRead(button_pin2);
  
  last_time_sensing = millis();
 
}

void loop()
{
  
  bool reading1 = digitalRead(button_pin1);
  bool reading2 = digitalRead(button_pin2);
  
  if (last_reading!= (reading1 | reading2)){
      last_debounce_time = millis();
      published = false;
  }
  
  //if the button value has not changed for the debounce delay, we know its stable
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    digitalWrite(led_pin, (reading1 | reading2));
    pushed_msg.data = (reading1 | reading2);
    pub_button.publish(&pushed_msg);
    published = true;
  }

  last_reading = (reading1 | reading2);
  
  if( abs(last_time_sensing*1.0 - millis()*1.0) > (20.0*60.0*1000.0) ) // every 20 minutes
  {
	// Reading temperature or humidity takes about 250 milliseconds!
        // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
        float h = 0.0;//dht.readHumidity(); // This sensor is to slow to work with ROS :(
        // Read temperature as Celsius (the default)
        float t = 0.0;//dht.readTemperature(); // This sensor is to slow to work with ROS :(
        
        // Get a new sensor event  
        sensors_event_t event;
        bmp.getEvent(&event);
        
        float temperature = t;
        
        // Display the results (barometric pressure is measure in hPa) 
        if (event.pressure)
        {
           // First we get the current temperature from the BMP085 
           bmp.getTemperature(&temperature);
           
           pressure_msg.data = event.pressure;
           pub_pressure.publish(&pressure_msg);
           temperature_msg.data = temperature;
           pub_temperature.publish(&temperature_msg);
        }
        
        
        humidity_msg.data = h;
        //pub_humidity.publish(&humidity_msg);
        
	last_time_sensing = millis();
  }

  nh.spinOnce();
}
