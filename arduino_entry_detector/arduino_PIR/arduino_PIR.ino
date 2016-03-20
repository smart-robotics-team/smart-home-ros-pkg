/* 
 * Button Example for Rosserial
 */

#include <ros.h>
#include <std_msgs/Bool.h>


ros::NodeHandle nh;

std_msgs::Bool pushed_msg;
ros::Publisher pub_button("pushed", &pushed_msg);

const int button_pin1 = 2;
const int button_pin2 = 3;
const int led_pin = 13;

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

void setup()
{
  nh.initNode();
  nh.advertise(pub_button);
  
  //initialize an LED output pin 
  //and a input pin for our push button
  pinMode(led_pin, OUTPUT);
  pinMode(button_pin1, INPUT);
  pinMode(button_pin2, INPUT);
  
  //Enable the pullup resistor on the button
  //digitalWrite(button_pin, HIGH);
  
  //The button is a normally button
  last_reading = digitalRead(button_pin1) | digitalRead(button_pin2);
 
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
  
  nh.spinOnce();
}
