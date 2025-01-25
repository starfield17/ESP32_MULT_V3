#include <Arduino.h>
#include <ESP32Encoder.h>
#include <U8g2lib.h>
#include <Thread.h>
#include <PID_v1.h>
#include <JC_Button.h>
#include <ezBuzzer.h>
#include <Adafruit_MPU6050.h>

//===== 配置头文件 Config.h =====//
#define DEBUG 0
#define MOTOR_ON 0

// 引脚配置
namespace Pins {
    const uint8_t 
        S1 = 1, S2 = 2,
        BUTTON1 = 37, BUTTON2 = 36, BUTTON3 = 35, ENCODER_KEY = 42,
        MOTOR_AIN1 = 39, MOTOR_AIN2 = 38, MOTOR_BIN1 = 41, MOTOR_BIN2 = 42,
        MOTOR_INA1 = 4, MOTOR_INA2 = 6, MOTOR_INB1 = 5, MOTOR_INB2 = 7,
        OLED_SCL = 47, OLED_SDA = 21,
        MPU_SDA = 18, MPU_SCL = 17,
        BEEP = 16;
}

// PID参数
struct PIDParams {
    double Kp, Ki, Kd;
    PIDParams(double p, double i, double d) : Kp(p), Ki(i), Kd(d) {}
};

//===== 电机控制类 =====//
class MotorController {
private:
    ESP32Encoder encoder;
    PID pid;
    uint8_t pwmPin;
    double setpoint, input, output;
    
public:
    MotorController(int pinA, int pinB, int pwmPin, PIDParams params)
        : pwmPin(pwmPin), 
          pid(&input, &output, &setpoint, params.Kp, params.Ki, params.Kd, DIRECT) {
        encoder.attachHalfQuad(pinA, pinB);
        ledcSetup(pwmPin, 1000, 8);
        ledcAttachPin(pwmPin, pwmPin);
        pid.SetMode(AUTOMATIC);
    }

    void update() {
        input = encoder.getCount();
        pid.Compute();
        ledcWrite(pwmPin, output);
    }

    void setSetpoint(double sp) { setpoint = sp; }
    void setTunings(double Kp, double Ki, double Kd) { pid.SetTunings(Kp, Ki, Kd); }
};

//===== 输入处理类 =====//
class InputHandler {
private:
    Button btn1, btn2, btn3, encoderBtn;
    ESP32Encoder encoder;
    bool lastBtn1 = false, lastBtn2 = false, lastBtn3 = false, lastEncoderBtn = false;
    
public:
    struct Event {
        bool btn1Pressed : 1;
        bool btn2Pressed : 1;
        bool btn3Pressed : 1;
        bool encoderPressed : 1;
        int8_t encoderDelta;
    };

    InputHandler() : 
        btn1(Pins::BUTTON1), btn2(Pins::BUTTON2), btn3(Pins::BUTTON3),
        encoderBtn(Pins::ENCODER_KEY) {
        ESP32Encoder::useInternalWeakPullResistors = UP;
        encoder.attachHalfQuad(Pins::S1, Pins::S2);
        btn1.begin();
        btn2.begin();
        btn3.begin();
        encoderBtn.begin();
    }

    Event poll() {
        Event e{};
        btn1.read(); btn2.read(); btn3.read(); encoderBtn.read();
        
        e.btn1Pressed = btn1.wasReleased();
        e.btn2Pressed = btn2.wasReleased();
        e.btn3Pressed = btn3.wasReleased();
        e.encoderPressed = encoderBtn.wasReleased();
        
        int64_t newPos = encoder.getCount();
        e.encoderDelta = newPos - encoder.getCount();
        encoder.clearCount();
        
        return e;
    }
};

//===== 显示管理类 =====//
class DisplayManager {
private:
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
    
public:
    DisplayManager() : u8g2(U8G2_R0, U8X8_PIN_NONE, Pins::OLED_SCL, Pins::OLED_SDA) {
        u8g2.begin();
        u8g2.setFont(u8g2_font_ncenB08_tr);
    }

    void clear() { u8g2.clearBuffer(); }
    void drawMenu(int selectedItem) {
        // 菜单绘制逻辑
    }
    void drawPIDParams(float Kp, float Ki, float Kd) {
        // PID参数显示
    }
    void flush() { u8g2.sendBuffer(); }
};

//===== 陀螺仪管理类 =====//
class GyroManager {
private:
    Adafruit_MPU6050 mpu;
    
public:
    bool begin() {
        Wire1.begin(Pins::MPU_SDA, Pins::MPU_SCL);
        if(!mpu.begin(0x68, &Wire1)) return false;
        mpu.setSampleRateDivisor(19);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        return true;
    }

    sensors_event_t getGyroData() {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        return g;
    }
};

//===== 主系统类 =====//
class RobotSystem {
private:
    MotorController motorA, motorB;
    InputHandler input;
    DisplayManager display;
    GyroManager gyro;
    ezBuzzer buzzer{Pins::BEEP};
    
    enum MenuState { MAIN_MENU, SET_SPEED, SET_PID, VIEW_GYRO };
    MenuState currentMenu = MAIN_MENU;
    
public:
    RobotSystem() : 
        motorA(Pins::MOTOR_INA1, Pins::MOTOR_INA2, Pins::MOTOR_AIN1, PIDParams(2.0,5.0,1.0)),
        motorB(Pins::MOTOR_INB1, Pins::MOTOR_INB2, Pins::MOTOR_BIN1, PIDParams(2.0,5.0,1.0)) {
        if(!gyro.begin()) Serial.println("MPU6050 init failed!");
    }

    void updateDisplay() {
        // 根据currentMenu状态更新显示
    }

    void handleInput() {
        auto event = input.poll();
        // 处理输入事件
    }

    void run() {
        motorA.update();
        motorB.update();
        buzzer.loop();
        updateDisplay();
    }
};

//===== 线程与初始化 =====//
RobotSystem robot;
ThreadController controller;

void systemUpdate() { robot.run(); }

void setup() {
    Serial.begin(9600);
    initializeThread(systemUpdate, 50);  // 50ms间隔
    // 其他线程初始化...
}

void loop() {
    controller.run();
}
