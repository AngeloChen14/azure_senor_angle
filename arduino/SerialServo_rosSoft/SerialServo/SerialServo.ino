
//For future reference, 这个代码是完整可以用的，由于ros本身的代码限制,altsoftserail在ros_lib/ArduinoHardware.h中调用
#define ROS_SOFTSERIAL 1

#include <ros.h>
#include <std_msgs/Float64.h>

#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 以A为高八位 B为低八位 合并为16位整形

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

//#define LOBOT_DEBUG 1  /*调试是用，打印调试数据*/

byte LobotCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LobotSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  SerialX.write(buf, 10);
}

void LobotSerialServoStopMove(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_MOVE_STOP;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
}

void LobotSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  SerialX.write(buf, 10);
}
void LobotSerialServoLoad(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = LobotCheckSum(buf);
  
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LobotSerialServoUnload(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = LobotCheckSum(buf);
  
  SerialX.write(buf, 7);
  
#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}


int LobotSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX.available()) {
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == LOBOT_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {
        
#ifdef LOBOT_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (LobotCheckSum(recvBuf) == recvBuf[dataCount - 1]) {
          
#ifdef LOBOT_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}


int LobotSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
int LobotSerialServoReadVin(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_VIN_READ;
  buf[5] = LobotCheckSum(buf);

#ifdef LOBOT_DEBUG
  Serial.println("LOBOT SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (LobotSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;

#ifdef LOBOT_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

#define PWM_RANGE 380  //pwm range between 0deg and 90deg(mid-pos)
#define PWM_MIDPOS 500  //pwm length for mid pos
#define CONTROL_INTERVAL 100    //Loop interval(ms),for servo control
#define FEEDBACK_INTERVAL 20    //Loop interval(ms),for feedback
#define LOST_INTERVAL 5000    //Lost cmd waiting interval(ms),for reset MIDPOS
#define M_PI 3.14159265358979323846
#define ID   1

template <typename type>
type sign(type value) {
 return type((value>0)-(value<0));
}

ros::NodeHandle  nh;
std_msgs::Float64 fb_msg;
void sub_cb(const std_msgs::Float64& cmd_msg);
ros::Publisher pub("/arduino/feedback", &fb_msg);
ros::Subscriber<std_msgs::Float64> sub("/camera_angle", sub_cb);
unsigned long control_timer;
unsigned long feedback_timer;
unsigned long last_update_time;

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  nh.getHardware()->setBaud(19200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  Serial.begin(115200);
  delay(1000);
  LobotSerialServoMove(Serial, ID, PWM_MIDPOS, 200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int current_PWM;
  if(millis()- feedback_timer > FEEDBACK_INTERVAL){
    feedback_timer = millis();
    current_PWM = LobotSerialServoReadPosition(Serial, ID);
    if(current_PWM!= 341 && current_PWM<=PWM_MIDPOS+PWM_RANGE && current_PWM>=PWM_MIDPOS-PWM_RANGE)
    {
      fb_msg.data = (float)(current_PWM - PWM_MIDPOS)*M_PI/(2*PWM_RANGE);
      pub.publish(&fb_msg);
    }    
  }
  if(millis()- last_update_time> LOST_INTERVAL){
    LobotSerialServoMove(Serial, ID, PWM_MIDPOS, 0);
  }
  nh.spinOnce();
}

void sub_cb( const std_msgs::Float64& cmd_msg){
  int target_PWM;
  if(millis()- control_timer > CONTROL_INTERVAL){
    control_timer = millis();
    digitalWrite(13, HIGH-digitalRead(13));  //toggle led
    // float target_angle;
    // target_angle = cmd_msg.data;
    target_PWM = constrain(PWM_MIDPOS + round(cmd_msg.data/M_PI*2*PWM_RANGE), PWM_MIDPOS-PWM_RANGE, PWM_MIDPOS+PWM_RANGE);
//    target_PWM = constrain(PWM_MIDPOS + round(cmd_msg.data/90.0*PWM_RANGE), PWM_MIDPOS-PWM_RANGE, PWM_MIDPOS+PWM_RANGE);
    LobotSerialServoMove(Serial, ID, target_PWM, 0);
  }
  last_update_time = millis();

}
