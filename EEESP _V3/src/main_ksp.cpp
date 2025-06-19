#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Thread.h>
#include <ThreadController.h>
#include <JC_Button.h>
#include <ezBuzzer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>

// DEBUG MODE
#define DEBUG 0
#define MOTOR_ON 0
// 引脚定义
#define S1 1
#define S2 2
#define BUTTON1 37
#define BUTTON2 36
#define BUTTON3 35
#define ENCODER_KEY 42
// 【注意】电机引脚现在由软件PWM控制
#define AIN1 39  // 加热丝控制
#define AIN2 38
#define BIN1 41  // 风扇控制
#define BIN2 42
#define INA1 4
#define INA2 6
#define INB1 5
#define INB2 7
#define OLED_SCL 47
#define OLED_SDA 21
// DS18B20温度传感器定义
#define DS18B20_DQ 45  // DS18B20数据引脚
//陀螺仪
#define MPU_INT 8
#define MPU_SCL 17
#define MPU_SDA 18
//NRF
#define SET 15
#define BEEP 16
#define IRQ 3
#define MISO 46
#define MOSI 9
#define SCK 10
#define CSN 11
#define CE 12

// 参数
/*******************************************/
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/OLED_SCL, /* data=*/OLED_SDA);
bool b1 = 0, b2 = 0, b3 = 0, e1 = 0, lt = 0, rt = 0;
bool *pb1 = &b1, *pb2 = &b2, *pb3 = &b3, *pe1 = &e1, *plt = &lt, *prt = &rt;
int pwmValue1 = 0;  // 加热丝PWM占空比 0-255 (0-100%)
int pwmValue2 = 0;  // 风扇PWM占空比 0-255 (0-100%)
sensors_event_t a, g, temp; // 陀螺仪
ezBuzzer buzzer(BEEP);

// DS18B20温度传感器
OneWire oneWire(DS18B20_DQ);
DallasTemperature sensors(&oneWire);

// PID控制相关变量
double currentTemp = 0;      // 当前温度
double targetTemp = 25.0;    // 目标温度
double pidOutput = 0;        // PID输出
double Kp = 10, Ki = 0.5, Kd = 1;  // PID参数
PID myPID(&currentTemp, &pidOutput, &targetTemp, Kp, Ki, Kd, DIRECT);

// 温度读取线程
Thread TEMP_READ = Thread();
void temp_read_control()
{
  sensors.requestTemperatures();
  currentTemp = sensors.getTempCByIndex(0);
  
  // 检查温度读取是否有效
  if (currentTemp == DEVICE_DISCONNECTED_C) {
    currentTemp = 0;
  }
}

/*******************************************/
// 【修改】电机软件PWM控制线程，加入PID控制
Thread MOTOR_PWM = Thread();
ESP32Encoder Motorencoder1;
ESP32Encoder Motorencoder2;

void motor_pwm_control()
{
  static unsigned long lastMicros = 0;
  const int pwmPeriod_us = 2040; // ~490Hz
  
  // PID计算
  myPID.Compute();
  
  // 根据PID输出调整PWM值
  pwmValue1 = constrain(pidOutput, 0, 255);  // 加热丝PWM
  
  // 风扇控制逻辑：温度越高，风扇转速越快
  if (currentTemp > targetTemp + 2) {
    pwmValue2 = 255;  // 全速
  } else if (currentTemp > targetTemp) {
    pwmValue2 = 128;  // 半速
  } else {
    pwmValue2 = 0;    // 停止
  }
  
  // 根据pwmValue (0-255) 计算高电平持续时间
  unsigned int onTime1_us = (pwmValue1 * pwmPeriod_us) / 255;
  unsigned int onTime2_us = (pwmValue2 * pwmPeriod_us) / 255;
  
  unsigned long currentMicros = micros();
  
  // 周期结束，重置计时器
  if (currentMicros - lastMicros >= pwmPeriod_us) {
    lastMicros = currentMicros;
  }
  
  // 根据占空比设置加热丝的引脚状态
  if (currentMicros - lastMicros < onTime1_us) {
    digitalWrite(AIN1, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
  }
  
  // 根据占空比设置风扇的引脚状态
  if (currentMicros - lastMicros < onTime2_us) {
    digitalWrite(BIN1, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
  }
}

/*******************************************/
// 菜单项
struct Circle
{
  int number;
  Circle *next;
  Circle *prev;
};
Circle pointer0 = {0};
Circle pointer1 = {1};
Circle pointer2 = {2};
Circle pointer3 = {3};
void init_Circle()
{
  pointer0.next = &pointer1;
  pointer0.prev = &pointer3;
  pointer1.next = &pointer2;
  pointer1.prev = &pointer0;
  pointer2.next = &pointer3;
  pointer2.prev = &pointer1;
  pointer3.next = &pointer0;
  pointer3.prev = &pointer2;
}
Circle *CurrentPointer = &pointer0;
int Flag = -1, Set = 0;

/*******************************************/
// 编码器_按钮线程
/*******************************************/
Thread SCAN = Thread();
ESP32Encoder encoder;
int64_t position = 0;
int64_t position_old = 0;
int64_t newPosition = 0;
Button button1(BUTTON1);
Button button2(BUTTON2);
Button button3(BUTTON3);
Button button_encoder(ENCODER_KEY);
void KEY_ENCODER()
{
  button1.read();
  button2.read();
  button3.read();
  button_encoder.read();
  if (button1.wasReleased())
  {
    *pb1 = true;
  }
  if (button2.wasReleased())
  {
    *pb2 = true;
  }
  if (button3.wasReleased())
  {
    *pb3 = true;
  }
  if (button_encoder.wasReleased())
  {
    *pe1 = true;
  }
  newPosition = encoder.getCount(); // 读取编码器的位置
  if (newPosition > position)
  {
    *prt = true;
#if DEBUG
    Serial.println("顺时针");
#endif
  }
  else if (newPosition < position)
  {
    *plt = true;
#if DEBUG
    Serial.println("逆时针");
#endif
  }
  position_old = position;
  position = newPosition;
}
/*******************************************/

// 显示器线程
/*******************************************/
Thread Display = Thread();
void display()
{
  if (Flag == -1)
  {
    if (*prt)
    {
      CurrentPointer = CurrentPointer->next;
      rt = !rt;
      u8g2.clearBuffer();
    }
    if (*plt)
    {
      CurrentPointer = CurrentPointer->prev;
      lt = !lt;
      u8g2.clearBuffer();
    }
    if (*pb1)
    {
      CurrentPointer = CurrentPointer->next;
      b1 = !b1;
      u8g2.clearBuffer();
    }
    if (*pb3)
    {
      CurrentPointer = CurrentPointer->prev;
      b3 = !b3;
      u8g2.clearBuffer();
    }
    u8g2.drawStr(0, 10 + 2 * CurrentPointer->number * u8g2.getFontAscent(), ">>");
    u8g2.setFont(u8g2_font_ncenB08_tr); // 设置字体
    u8g2.drawStr(20, 10 + 2 * 0 * u8g2.getFontAscent(), "set PWM");
    u8g2.drawStr(20, 10 + 2 * 1 * u8g2.getFontAscent(), "LOOK XYZ");
    u8g2.drawStr(20, 10 + 2 * 2 * u8g2.getFontAscent(), "Temp Control");  // 温度控制
    u8g2.sendBuffer(); // 将缓冲区内容发送到显示屏
    switch (CurrentPointer->number)
    {
      break;
    case 0:
      if (b2)
      {
        b2 = !b2;
        Flag = 0;
        u8g2.clearBuffer();
        u8g2.sendBuffer();
      }
      break;
    case 1:
      if (b2)
      {
        b2 = !b2;
        Flag = 1;
        u8g2.clearBuffer();
        u8g2.sendBuffer();
      }
      break;
    case 2:
      if (b2)
      {
        b2 = !b2;
        Flag = 2;
        u8g2.clearBuffer();
        u8g2.sendBuffer();
      }
      break;
    default:
      break;
    }
  }
  if (Flag == 0)
  {
    if (*plt)
    {
      u8g2.clearBuffer();
      lt = !lt;
      pwmValue1 -= 5;  // 每次减少约2%
      pwmValue2 -= 5;
      if(pwmValue1 < 0) pwmValue1 = 0;
      if(pwmValue2 < 0) pwmValue2 = 0;
    }
    else if (*prt)
    {
      u8g2.clearBuffer();
      rt = !rt;
      pwmValue1 += 5;  // 每次增加约2%
      pwmValue2 += 5;
      if(pwmValue1 > 255) pwmValue1 = 255;
      if(pwmValue2 > 255) pwmValue2 = 255;
    }
    if (*pe1)
    {
      Flag = -1;
      *pe1 = !*pe1;
      u8g2.clearBuffer();
      u8g2.sendBuffer();
    }
    u8g2.setFont(u8g2_font_ncenB08_tr); // 设置字体
    u8g2.drawStr(0, 10 + 0 * 1 * u8g2.getFontAscent(), "CHANGE_PWM_DUTY");
    u8g2.drawStr(0, 10 + 1 * 2 * u8g2.getFontAscent(), "PWM1=");
    u8g2.drawStr(0, 10 + 2 * 2 * u8g2.getFontAscent(), "PWM2=");
    
    // 显示百分比
    int percent1 = (pwmValue1 * 100) / 255;
    int percent2 = (pwmValue2 * 100) / 255;
    u8g2.drawStr(50, 10 + 1 * 2 * u8g2.getFontAscent(), (String(percent1) + "%").c_str());
    u8g2.drawStr(50, 10 + 2 * 2 * u8g2.getFontAscent(), (String(percent2) + "%").c_str());
    u8g2.sendBuffer(); // 将缓冲区内容发送到显示屏
  }
  if (Flag == 1)
  {
    if (*plt)
    {
      u8g2.clearBuffer();
      lt = !lt;
    }
    else if (*prt)
    {
      u8g2.clearBuffer();
      rt = !rt;
    }
    if (*pe1)
    {
      Flag = -1;
      *pe1 = !*pe1;
      u8g2.clearBuffer();
      u8g2.sendBuffer();
    }
    u8g2.clearBuffer();
    u8g2.drawStr(40, 10 + 0 * 2 * u8g2.getFontAscent(), String(g.gyro.x).c_str());
    u8g2.drawStr(40, 10 + 1 * 2 * u8g2.getFontAscent(), String(g.gyro.y).c_str());
    u8g2.drawStr(40, 10 + 2 * 2 * u8g2.getFontAscent(), String(g.gyro.z).c_str());
    u8g2.drawStr(20, 10 + 2 * 0 * u8g2.getFontAscent(), "X=");
    u8g2.drawStr(20, 10 + 2 * 1 * u8g2.getFontAscent(), "Y=");
    u8g2.drawStr(20, 10 + 2 * 2 * u8g2.getFontAscent(), "Z=");
    u8g2.sendBuffer(); // 将缓冲区内容发送到显示屏
  }
  if (Flag == 2)
  {
    // 温度控制界面
    if (*plt)
    {
      u8g2.clearBuffer();
      lt = !lt;
      targetTemp -= 0.5;  // 每次减少0.5度
      if(targetTemp < 0) targetTemp = 0;
    }
    else if (*prt)
    {
      u8g2.clearBuffer();
      rt = !rt;
      targetTemp += 0.5;  // 每次增加0.5度
      if(targetTemp > 100) targetTemp = 100;
    }
    if (*pe1)
    {
      Flag = -1;
      *pe1 = !*pe1;
      u8g2.clearBuffer();
      u8g2.sendBuffer();
    }
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(10, 10, "Temperature Control");
    
    // 显示当前温度
    char tempStr[20];
    dtostrf(currentTemp, 4, 1, tempStr);
    u8g2.drawStr(10, 25, "Current: ");
    u8g2.drawStr(70, 25, tempStr);
    u8g2.drawStr(100, 25, "C");
    
    // 显示目标温度
    dtostrf(targetTemp, 4, 1, tempStr);
    u8g2.drawStr(10, 40, "Target: ");
    u8g2.drawStr(70, 40, tempStr);
    u8g2.drawStr(100, 40, "C");
    
    // 显示加热器和风扇状态
    int heaterPercent = (pwmValue1 * 100) / 255;
    int fanPercent = (pwmValue2 * 100) / 255;
    u8g2.drawStr(10, 55, "Heat:");
    u8g2.drawStr(50, 55, (String(heaterPercent) + "%").c_str());
    u8g2.drawStr(80, 55, "Fan:");
    u8g2.drawStr(110, 55, (String(fanPercent) + "%").c_str());
    
    u8g2.sendBuffer();
  }
}

/*******************************************/
/*******************************************/
Adafruit_MPU6050 mpu;
Thread State_Machine = Thread();
void State_Machine_run()
{
  Serial.print("1");
  char incomingByte = Serial.read();
  if (incomingByte =='1') { // 检查读取的数据是否有效
  buzzer.beep(500);
  }
  mpu.getEvent(&a, &g, &temp);
  buzzer.loop();
}
/*******************************************/
ThreadController controller = ThreadController();
// 初始化
void initializeThread(Thread& thread, void (*runFunction)(), int interval) {
    thread.onRun(runFunction);
    thread.setInterval(interval);
    controller.add(&thread);
}
void setup()
{
  Serial.begin(9600); // 确保波特率与串口监视器一致
  
  // 初始化DS18B20
  sensors.begin();
  sensors.setResolution(12);  // 设置12位分辨率
  
  // 初始化PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // PWM范围
  myPID.SetSampleTime(1000);      // 1秒采样时间
  
  // 【新增】初始化电机引脚为输出模式
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  // 设置电机方向，AIN2和BIN2保持低电平，通过AIN1和BIN1的PWM信号控制速度
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);

  // init buttons
  Wire1.begin(MPU_SDA, MPU_SCL);
  Wire1.setClock(400000);
  // 初始化MPU6050
  if (!mpu.begin(0x68, &Wire1)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");
  
  mpu.setSampleRateDivisor(19);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  button1.begin();
  button2.begin();
  button3.begin();
  button_encoder.begin();
  
  init_Circle();
  
  // init encoder
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachHalfQuad(S1, S2);
  encoder.clearCount();
  Motorencoder1.attachHalfQuad(INA1, INA2);
  Motorencoder1.clearCount();
  Motorencoder2.attachHalfQuad(INB1, INB2);
  Motorencoder2.clearCount();
  
  // 初始化PWM值
  pwmValue1 = 0;
  pwmValue2 = 0;
  
  // init display
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  
  delay(1001);
  
  // init thread
  initializeThread(State_Machine, State_Machine_run, 40);
  initializeThread(Display, display, 80);
  initializeThread(SCAN, KEY_ENCODER, 100);
  initializeThread(MOTOR_PWM, motor_pwm_control, 0);
  initializeThread(TEMP_READ, temp_read_control, 1000);  // 每秒读取一次温度
}

void loop()
{
  controller.run();
}
