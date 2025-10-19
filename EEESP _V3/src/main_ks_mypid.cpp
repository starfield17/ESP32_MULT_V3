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
// #include <PID_v1.h> // <-- Removed PID_v1 library

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
// 【Note】Motor pins are now controlled by software PWM
#define AIN1 39  // Heating wire control
#define AIN2 38
#define BIN1 41  // Fan control
#define BIN2 42
#define INA1 4
#define INA2 6
#define INB1 5
#define INB2 7
#define OLED_SCL 47
#define OLED_SDA 21
// DS18B20 temperature sensor definition
#define DS18B20_DQ 45  // DS18B20 data pin
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

/*****************************************************************/
/********************* Custom PID Controller Class *************************/
/*****************************************************************/
class CustomPID
{
public:
    // Constructor
    CustomPID(double* input, double* output, double* setpoint,
              double Kp, double Ki, double Kd)
    {
        myOutput = output;
        myInput = input;
        mySetpoint = setpoint;
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;

        // Initialize variables
        lastInput = *myInput;
        integralTerm = 0;
        lastTime = millis() - sampleTime;
    }

    // Calculate PID output
    bool Compute()
    {
        unsigned long now = millis();
        unsigned long timeChange = (now - lastTime);

        if (timeChange >= sampleTime)
        {
            // Get input, output, and setpoint values
            double input = *myInput;
            double setpoint = *mySetpoint;
            
            // Calculate error
            double error = setpoint - input;
            
            // Calculate integral term (with anti-windup)
            integralTerm += (_Ki * error);
            if (integralTerm > outMax) integralTerm = outMax;
            else if (integralTerm < outMin) integralTerm = outMin;
            
            // Calculate derivative term
            double dInput = (input - lastInput);
            
            // Calculate total PID output
            double output = _Kp * error + integralTerm - _Kd * dInput;
            
            // Limit output range
            if (output > outMax) output = outMax;
            else if (output < outMin) output = outMin;
            
            *myOutput = output;
            
            // Save current state for next calculation
            lastInput = input;
            lastTime = now;
            return true;
        }
        return false;
    }

    // Set output limits
    void SetOutputLimits(double min, double max)
    {
        if (min >= max) return;
        outMin = min;
        outMax = max;
        
        // Ensure current output is within limits
        if (myOutput != nullptr) {
            if (*myOutput > outMax) *myOutput = outMax;
            else if (*myOutput < outMin) *myOutput = outMin;
        }

        // Ensure integral term is also within limits
        if (integralTerm > outMax) integralTerm = outMax;
        else if (integralTerm < outMin) integralTerm = outMin;
    }
    
    // Set sample time (ms)
    void SetSampleTime(int newSampleTime)
    {
        if (newSampleTime > 0)
        {
            sampleTime = (unsigned long)newSampleTime;
        }
    }
    
    // Set controller mode (for compatibility, but doesn't have actual effect in this implementation)
    void SetMode(int mode) {
        // In this simple implementation, we are always in "AUTOMATIC" mode
        // This function is mainly to keep the code structure consistent with the PID_v1 library
    }

private:
    double _Kp, _Ki, _Kd;           // PID gain parameters
    double *myInput;                   // Pointer to input variable (e.g., current temperature)
    double *myOutput;                  // Pointer to output variable (e.g., PWM value)
    double *mySetpoint;                // Pointer to setpoint variable (e.g., target temperature)
    
    unsigned long lastTime;            // Time of last calculation
    double integralTerm;               // Integral term accumulation
    double lastInput;                  // Last input value
    
    unsigned long sampleTime = 1000;   // Default sample time 1000ms
    double outMin = 0, outMax = 255;   // Default output value range
};


/*******************************************/
// Parameters
/*******************************************/
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE, /* clock=*/OLED_SCL, /* data=*/OLED_SDA);
bool b1 = 0, b2 = 0, b3 = 0, e1 = 0, lt = 0, rt = 0;
bool *pb1 = &b1, *pb2 = &b2, *pb3 = &b3, *pe1 = &e1, *plt = &lt, *prt = &rt;
int pwmValue1 = 0;  // Heating wire PWM duty cycle 0-255 (0-100%)
int pwmValue2 = 0;  // Fan PWM duty cycle 0-255 (0-100%)
sensors_event_t a, g, temp; // Gyroscope
ezBuzzer buzzer(BEEP);

// DS18B20 temperature sensor
OneWire oneWire(DS18B20_DQ);
DallasTemperature sensors(&oneWire);

// PID control related variables
double currentTemp = 0;      // Current temperature
double targetTemp = 25.0;    // Target temperature
double pidOutput = 0;        // PID output
double Kp = 10, Ki = 0.5, Kd = 1;  // PID parameters

// 【Modified】Use custom PID class to instantiate object
CustomPID myPID(&currentTemp, &pidOutput, &targetTemp, Kp, Ki, Kd);

// Temperature reading thread
Thread TEMP_READ = Thread();
void temp_read_control()
{
  sensors.requestTemperatures();
  double tempReading = sensors.getTempCByIndex(0);
  
  // Check if temperature reading is valid
  if (tempReading != DEVICE_DISCONNECTED_C) {
    currentTemp = tempReading;
  }
}

/*******************************************/
// 【Modified】Motor software PWM control thread, with PID control
Thread MOTOR_PWM = Thread();
ESP32Encoder Motorencoder1;
ESP32Encoder Motorencoder2;

void motor_pwm_control()
{
  static unsigned long lastMicros = 0;
  const int pwmPeriod_us = 2040; // ~490Hz
  
  // PID calculation
  myPID.Compute();
  
  // Adjust PWM value based on PID output
  pwmValue1 = constrain((int)pidOutput, 0, 255);  // Heating wire PWM
  
  // Fan synchronized control with heating wire to ensure air circulation
  // When heating, fan runs simultaneously to promote uniform heat distribution
  pwmValue2 = pwmValue1;  // Fan PWM synchronized with heating wire

  // Calculate high-level duration based on pwmValue (0-255)
  unsigned int onTime1_us = (pwmValue1 * pwmPeriod_us) / 255;
  unsigned int onTime2_us = (pwmValue2 * pwmPeriod_us) / 255;
  
  unsigned long currentMicros = micros();
  
  // Period end, reset timer
  if (currentMicros - lastMicros >= pwmPeriod_us) {
    lastMicros = currentMicros;
  }
  
  // Set heating wire pin state based on duty cycle
  if (currentMicros - lastMicros < onTime1_us) {
    digitalWrite(AIN1, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
  }
  
  // Set fan pin state based on duty cycle
  if (currentMicros - lastMicros < onTime2_us) {
    digitalWrite(BIN1, HIGH);
  } else {
    digitalWrite(BIN1, LOW);
  }
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
  newPosition = encoder.getCount(); // Read encoder position
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
    Serial.println("Counterclockwise");
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
    u8g2.setFont(u8g2_font_ncenB08_tr); // Set font
    u8g2.drawStr(20, 10 + 2 * 0 * u8g2.getFontAscent(), "set PWM");
    u8g2.drawStr(20, 10 + 2 * 1 * u8g2.getFontAscent(), "LOOK XYZ");
    u8g2.drawStr(20, 10 + 2 * 2 * u8g2.getFontAscent(), "Temp Control");  // Temperature control
    u8g2.sendBuffer(); // Send buffer content to display
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
      pwmValue1 -= 5;  // Decrease by about 2% each time
      pwmValue2 -= 5;
      if(pwmValue1 < 0) pwmValue1 = 0;
      if(pwmValue2 < 0) pwmValue2 = 0;
    }
    else if (*prt)
    {
      u8g2.clearBuffer();
      rt = !rt;
      pwmValue1 += 5;  // Increase by about 2% each time
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
    u8g2.setFont(u8g2_font_ncenB08_tr); // Set font
    u8g2.drawStr(0, 10 + 0 * 1 * u8g2.getFontAscent(), "CHANGE_PWM_DUTY");
    u8g2.drawStr(0, 10 + 1 * 2 * u8g2.getFontAscent(), "PWM1=");
    u8g2.drawStr(0, 10 + 2 * 2 * u8g2.getFontAscent(), "PWM2=");
    
    // Display percentage
    int percent1 = (pwmValue1 * 100) / 255;
    int percent2 = (pwmValue2 * 100) / 255;
    u8g2.drawStr(50, 10 + 1 * 2 * u8g2.getFontAscent(), (String(percent1) + "%").c_str());
    u8g2.drawStr(50, 10 + 2 * 2 * u8g2.getFontAscent(), (String(percent2) + "%").c_str());
    u8g2.sendBuffer(); // Send buffer content to display
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
    u8g2.sendBuffer(); // Send buffer content to display
  }
  if (Flag == 2)
  {
    // Temperature control interface
    if (*plt)
    {
      u8g2.clearBuffer();
      lt = !lt;
      targetTemp -= 0.5;  // Decrease by 0.5 degrees each time
      if(targetTemp < 0) targetTemp = 0;
    }
    else if (*prt)
    {
      u8g2.clearBuffer();
      rt = !rt;
      targetTemp += 0.5;  // Increase by 0.5 degrees each time
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
    
    // Display current temperature
    char tempStr[20];
    dtostrf(currentTemp, 4, 1, tempStr);
    u8g2.drawStr(10, 25, "Current: ");
    u8g2.drawStr(70, 25, tempStr);
    u8g2.drawStr(100, 25, "C");
    
    // Display target temperature
    dtostrf(targetTemp, 4, 1, tempStr);
    u8g2.drawStr(10, 40, "Target: ");
    u8g2.drawStr(70, 40, tempStr);
    u8g2.drawStr(100, 40, "C");
    
    // Display heater and fan status
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
  if (incomingByte =='1') { // Check if read data is valid
  buzzer.beep(500);
  }
  mpu.getEvent(&a, &g, &temp);
  buzzer.loop();
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
  Serial.begin(9600); // Ensure baud rate matches serial monitor
  
  // Initialize DS18B20
  sensors.begin();
  sensors.setResolution(12);  // Set 12-bit resolution
  
  // Initialize PID
  myPID.SetMode(0); // Parameter here is just for compatibility
  myPID.SetOutputLimits(0, 255);  // PWM range
  myPID.SetSampleTime(1000);      // 1 second sample time
  
  // 【Added】Initialize motor pins as output mode
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  // Set motor direction, AIN2 and BIN2 remain low, control speed through AIN1 and BIN1 PWM signals
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN2, LOW);

  // init buttons
  Wire1.begin(MPU_SDA, MPU_SCL);
  Wire1.setClock(400000);
  // Initialize MPU6050
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
  
  // Initialize PWM values
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
  initializeThread(TEMP_READ, temp_read_control, 1000);  // Read temperature once per second
}

void loop()
{
  controller.run();
}
