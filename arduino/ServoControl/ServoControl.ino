/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float64.h>

#define PWM_PIN 9       //digital pwm output pin number
#define PWM_RANGE 940  //pwm us range between 0deg and 90deg(mid-pos)
#define PWM_MIDPOS 1500  //pwm length for mid pos
#define LOOP_INTERVAL 10    //Loop interval(ms),for servo control and feedback
#define RAMP_SLOPE 10   //PWM change per loop, 40/1000*90 = 3.6deg/10ms = 36deg/0.1s = 360deg/s
#define M_PI 3.14159265358979323846
template <typename type>
type sign(type value) {
 return type((value>0)-(value<0));
}

ros::NodeHandle  nh;
std_msgs::Float64 fb_msg;
ros::Publisher pub("/arduino/feedback", &fb_msg);
void sub_cb(const std_msgs::Float64& cmd_msg);
ros::Subscriber<std_msgs::Float64> sub("/camera_angle", sub_cb);
//float target_angle;
int target_PWM = 1500;
int current_PWM = 1500;
Servo servo;

float servo_control(){
  float current_angle;
  if(target_PWM!=current_PWM){
    if(abs(target_PWM - current_PWM) < RAMP_SLOPE)
        current_PWM  = target_PWM;
    else
        current_PWM += RAMP_SLOPE*sign(target_PWM - current_PWM); 
  }
  servo.writeMicroseconds(current_PWM);
  current_angle = (float)(current_PWM - PWM_MIDPOS)*M_PI/(2*PWM_RANGE);
  return current_angle;
}

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  
  servo.attach(PWM_PIN); //attach it to pin 9
}

unsigned long timer;
void loop(){
  if(millis()- timer > LOOP_INTERVAL){  //10ms loop -> 100Hz control and feedback
    fb_msg.data = servo_control();
    pub.publish(&fb_msg);
    timer = millis();
  }
  nh.spinOnce();
}

void sub_cb( const std_msgs::Float64& cmd_msg){
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led
//  target_angle = cmd_msg.data;
  target_PWM = constrain(PWM_MIDPOS + int(cmd_msg.data/M_PI*2*PWM_RANGE), PWM_MIDPOS-PWM_RANGE, PWM_MIDPOS+PWM_RANGE);
}
