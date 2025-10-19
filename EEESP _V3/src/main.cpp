#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Thread.h>
#include <ThreadController.h>
#include <PID_v1.h>
#include <JC_Button.h>
#include <ezBuzzer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// DEBUG MODE
#define DEBUG 0
#define MOTOR_ON 0
// Pin definitions
#define S1 1
#define S2 2
#define BUTTON1 37
#define BUTTON2 36
#define BUTTON3 35
#define ENCODER_KEY 42
#define BIN1 41
#define BIN2 42
#define AIN1 39
#define AIN2 38
#define INA1 4
#define INA2 6
#define INB1 5
#define INB2 7
#define OLED_SCL 47
#define OLED_SDA 21
//Gyroscope
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
// Parameters
/*******************************************/
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/OLED_SCL, /* data=*/OLED_SDA);
bool b1 = 0, b2 = 0, b3 = 0, e1 = 0, lt = 0, rt = 0;
bool *pb1 = &b1, *pb2 = &b2, *pb3 = &b3, *pe1 = &e1, *plt = &lt, *prt = &rt;
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
int pwmFrequency = 1000;
  sensors_event_t a, g, temp; // Gyroscope
ezBuzzer buzzer(BEEP);
/*******************************************/
// pid motor control thread
Thread MOTOR_CONTROL = Thread();
ESP32Encoder Motorencoder1;
ESP32Encoder Motorencoder2;
// Create PID objects
PID myPID1(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID myPID2(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);
void motor_control()
{

  Input1 = Motorencoder1.getCount();
  Input2 = Motorencoder2.getCount();
  // Calculate PID output
  myPID1.Compute();
  myPID2.Compute();
  // Control motor speed
  ledcWrite(AIN1, Output1);
  ledcWrite(BIN1, Output2);
}
/*******************************************/
// Menu items
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
// Encoder_Button thread
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
  newPosition = encoder.getCount(); // Read the encoder position
  if (newPosition > position)
  {
    *prt = true;
#if DEBUG
    Serial.println("Clockwise");
#endif
  }
  else if (newPosition < position)
  {
    *plt = true;
#if DEBUG
    Serial.println("Counter-clockwise");
#endif
  }
  position_old = position;
  position = newPosition;
}
/*******************************************/

// Display thread
/*******************************************/
Thread Display = Thread();
void display()
{
  if (Flag == -1)
  {
   // Serial.println("F=-1");
    if (*prt)
    {
      CurrentPointer = CurrentPointer->next;
      rt = !rt;
      u8g2.clearBuffer();
      // Serial.println("P+");
    }
    if (*plt)
    {
      CurrentPointer = CurrentPointer->prev;
      lt = !lt;
      u8g2.clearBuffer();
      // Serial.println("P-");
    }
    if (*pb1)
    {
      CurrentPointer = CurrentPointer->next;
      b1 = !b1;
      u8g2.clearBuffer();
     // Serial.println("P+");
    }
    if (*pb3)
    {
      CurrentPointer = CurrentPointer->prev;
      b3 = !b3;
      u8g2.clearBuffer();
     // Serial.println("P-");
    }
    u8g2.drawStr(0, 10 + 2 * CurrentPointer->number * u8g2.getFontAscent(), ">>");
    u8g2.setFont(u8g2_font_ncenB08_tr); // Set font
    u8g2.drawStr(20, 10 + 2 * 0 * u8g2.getFontAscent(), "set Speed");
    u8g2.drawStr(20, 10 + 2 * 1 * u8g2.getFontAscent(), "LOOK XYZ");
    u8g2.drawStr(20, 10 + 2 * 2 * u8g2.getFontAscent(), "set PID");
    u8g2.sendBuffer(); // Send buffer content to display
    switch (CurrentPointer->number)
    {
      break;
    case 0:
      if (b2)
      {
        b2 = !b2;
        Flag = 0;
      //  Serial.println("b2");
        u8g2.clearBuffer();
        u8g2.sendBuffer();
      }
      break;
    case 1:
      if (b2)
      {
        b2 = !b2;
        Flag = 1;
       // Serial.println("b2");
        u8g2.clearBuffer();
        u8g2.sendBuffer();
      }
      break;
    case 2:
      if (b2)
      {
        b2 = !b2;
        Flag = 2;
      //  Serial.println("b2");
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
    //  Serial.println("s-");
      lt = !lt;
      Setpoint1 -= 50;
      Setpoint2 -= 50;
    }
    else if (*prt)
    {
      u8g2.clearBuffer();
    //  Serial.println("s+");
      rt = !rt;
      Setpoint1 += 50;
      Setpoint2 += 50;
    }
    if (*pe1)
    {
      Flag = -1;
      *pe1 = !*pe1;
      u8g2.clearBuffer();
      u8g2.sendBuffer();
    }
    u8g2.setFont(u8g2_font_ncenB08_tr); // Set font
    //Serial.println("F=0");
    u8g2.drawStr(0, 10 + 0 * 1 * u8g2.getFontAscent(), "CHANGE_the_Speed");
    u8g2.drawStr(0, 10 + 1 * 2 * u8g2.getFontAscent(), "S1=");
    u8g2.drawStr(0, 10 + 2 * 2 * u8g2.getFontAscent(), "S2=");
    u8g2.drawStr(30, 10 + 1 * 2 * u8g2.getFontAscent(), String(Setpoint1).c_str());
    u8g2.drawStr(30, 10 + 2 * 2 * u8g2.getFontAscent(), String(Setpoint2).c_str());
    u8g2.sendBuffer(); // Send buffer content to display
  }
  if (Flag == 1)
  {
   // Serial.println("F=1");
    if (*plt)
    {
      u8g2.clearBuffer();
      lt = !lt;
      ;
    }
    else if (*prt)
    {
      u8g2.clearBuffer();
   //   Serial.println("s+");
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
    u8g2.sendBuffer(); // Send buffer content to display
  }
  if (Flag == 2)
  {
    if (*pb1)
    {
      CurrentPointer = CurrentPointer->next;
      b1 = !b1;
      u8g2.clearBuffer();
      //Serial.println("P-");
    }
    if (*pb3)
    {
      CurrentPointer = CurrentPointer->prev;
      b3 = !b3;
      u8g2.clearBuffer();
      //Serial.println("P+");
    }
    //Serial.println("F=2");
    if (Set == 0)
    {
      if (*pe1)
      {
        Flag = -1;
        *pe1 = !*pe1;
        u8g2.clearBuffer();
        u8g2.sendBuffer();
      }
    }
    else
    {
      if (*pe1)
      {
        Set = 0;
        *pe1 = !*pe1;
        u8g2.clearBuffer();
        u8g2.sendBuffer();
      }
    }
    switch (CurrentPointer->number)
    {
    case 0:
      if (b2)
      {
        b2 = !b2;
        Set = 1;
        //Serial.println("b2");
        u8g2.clearBuffer();
        u8g2.sendBuffer();
      }
      break;
    case 1:
      if (b2)
      {
        b2 = !b2;
        Set = 2;
       // Serial.println("b2");
        u8g2.clearBuffer();
        u8g2.sendBuffer();
      }
      break;
    case 2:
      if (b2)
      {
        b2 = !b2;
        Set = 3;
        //Serial.println("b2");
        u8g2.clearBuffer();
        u8g2.sendBuffer();
      }
      break;
    default:
      break;
    }
    switch (Set)
    {
    case 1:
      if (*prt)
      {
        Kp += 0.1;
        rt = !rt;
        u8g2.clearBuffer();
       // Serial.println("P+");
      }
      if (*plt)
      {
        Kp -= 0.1;
        lt = !lt;
        u8g2.clearBuffer();
       // Serial.println("P-");
      }
      if (*pe1)
      {
        *pe1 = !*pe1;
        u8g2.clearBuffer();
        u8g2.sendBuffer();
        break;
      }
    case 2:
      if (*prt)
      {
        Ki += 0.1;
        rt = !rt;
        u8g2.clearBuffer();
        //Serial.println("I+");
      }
      if (*plt)
      {
        Ki -= 0.1;
        lt = !lt;
        u8g2.clearBuffer();
        //Serial.println("I-");
      }
      if (*pe1)
      {
        *pe1 = !*pe1;
        u8g2.clearBuffer();
        u8g2.sendBuffer();
        break;
      }
    case 3:
      if (*prt)
      {
        Kd += 0.1;
        rt = !rt;
        u8g2.clearBuffer();
      //  Serial.println("D+");
      }
      if (*plt)
      {
        Kd -= 0.1;
        lt = !lt;
        u8g2.clearBuffer();
       // Serial.println("D-");
      }
    default:
      break;
    }

    u8g2.drawStr(0, 10 + 2 * CurrentPointer->number * u8g2.getFontAscent(), ">>");
    u8g2.setFont(u8g2_font_ncenB08_tr); // Set font
    u8g2.drawStr(40, 10 + 0 * 2 * u8g2.getFontAscent(), String(Kp).c_str());
    u8g2.drawStr(40, 10 + 1 * 2 * u8g2.getFontAscent(), String(Ki).c_str());
    u8g2.drawStr(40, 10 + 2 * 2 * u8g2.getFontAscent(), String(Kd).c_str());
    u8g2.drawStr(20, 10 + 2 * 0 * u8g2.getFontAscent(), "P=");
    u8g2.drawStr(20, 10 + 2 * 1 * u8g2.getFontAscent(), "I=");
    u8g2.drawStr(20, 10 + 2 * 2 * u8g2.getFontAscent(), "D=");
    u8g2.sendBuffer(); // Send buffer content to display
    myPID1.SetTunings(Kp, Ki, Kd);
    myPID2.SetTunings(Kp, Ki, Kd);
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
  if (incomingByte =='1') { // Check if the read data is valid
  buzzer.beep(500);
  }
  mpu.getEvent(&a, &g, &temp);
  
    buzzer.loop();
    // String dataToSend ="test";
    // String receivedData =Serial.readString();
    // Serial.println("Sent=="+dataToSend);
    // if (millis() % 2000 < 500) {
    // buzzer.beep(500);
    //   }
    
}
/*******************************************/
ThreadController controller = ThreadController();
// Initialization
void initializeThread(Thread& thread, void (*runFunction)(), int interval) {
    thread.onRun(runFunction);
    thread.setInterval(interval);
    controller.add(&thread);
}
void setup()
{
  Serial.begin(9600); // Ensure baud rate matches the serial monitor
  // init buttons
  Wire1.begin(MPU_SDA, MPU_SCL);
  Wire1.setClock(400000); // Set I2C frequency to 400kHz
  // Initialize MPU6050
  if (!mpu.begin(0x68, &Wire1)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");
  
 mpu.setSampleRateDivisor(19); // Sample rate = 1kHz / (1 + 19) = 50Hz

  // Set low-pass filter
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Set low-pass filter cutoff frequency to 21Hz

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  button1.begin();
  button2.begin();
  button3.begin();
  button_encoder.begin();
  // init pwm controller
  init_Circle();
  // init encoder
  ESP32Encoder::useInternalWeakPullResistors = UP; // Use internal pull-up resistors
  encoder.attachHalfQuad(S1, S2);                  // Pins 2 and 3 connect to the encoder's A and B signal outputs
  encoder.clearCount();                            // Clear the counter
  Motorencoder1.attachHalfQuad(INA1, INA2);
  Motorencoder1.clearCount();
  Motorencoder2.attachHalfQuad(INB1, INB2);
  Motorencoder2.clearCount();
  // init pid
  ledcSetup(AIN1, pwmFrequency, 8); // 5kHz PWM, 8-bit resolution
  ledcSetup(AIN2, pwmFrequency, 8); // 5kHz PWM, 8-bit resolution
  ledcSetup(BIN1, pwmFrequency, 8); // 5kHz PWM, 8-bit resolution
  ledcSetup(BIN2, pwmFrequency, 8); // 5kHz PWM, 8-bit resolution
  Setpoint1 = 1000;
  Setpoint2 = 1000;
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  // init display
  u8g2.begin();
  u8g2.clearBuffer();                 // Clear the buffer
  u8g2.setFont(u8g2_font_ncenB08_tr); // Set font
  // wait
  delay(1001);
  // init thread
initializeThread(State_Machine, State_Machine_run, 40);
initializeThread(Display, display, 80);
initializeThread(SCAN, KEY_ENCODER, 100);
initializeThread(MOTOR_CONTROL, motor_control, 50);
}
// Multi-thread start!
void loop()
{
  controller.run();
}
