#include <Arduino.h>
#include <ESP32Encoder.h>
#include <U8g2lib.h>
#include <PID_v1.h>
#include <JC_Button.h>
#include <ezBuzzer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// 硬件引脚定义（同原代码）
// 此处省略引脚定义，保持与原代码一致

//======================================
// 全局对象和数据结构
//======================================
QueueHandle_t inputQueue = xQueueCreate(10, sizeof(int));
SemaphoreHandle_t i2cMutex = xSemaphoreCreateMutex();

//======================================
// Class: MotorController
//======================================
class MotorController {
private:
    ESP32Encoder encoder1, encoder2;
    PID pid1, pid2;
    double setpoint1, setpoint2;
    double kp, ki, kd;
    
public:
    MotorController() : 
        pid1(&encoder1.getCount(), &output1, &setpoint1, kp, ki, kd, DIRECT),
        pid2(&encoder2.getCount(), &output2, &setpoint2, kp, ki, kd, DIRECT) {
        // 初始化代码
    }

    void init() {
        encoder1.attachHalfQuad(INA1, INA2);
        encoder2.attachHalfQuad(INB1, INB2);
        ledcSetup(AIN1, 1000, 8);
        // ...其他初始化
    }

    void update() {
        pid1.Compute();
        pid2.Compute();
        ledcWrite(AIN1, output1);
        ledcWrite(BIN1, output2);
    }

    void setPID(double p, double i, double d) {
        kp = p; ki = i; kd = d;
        pid1.SetTunings(kp, ki, kd);
        pid2.SetTunings(kp, ki, kd);
    }
} motorCtrl;

//======================================
// Class: InputHandler
//======================================
class InputHandler {
private:
    Button btn1, btn2, btn3, btnEnc;
    ESP32Encoder encoder;
    
public:
    InputHandler() : 
        btn1(BUTTON1), btn2(BUTTON2), 
        btn3(BUTTON3), btnEnc(ENCODER_KEY) {}

    void init() {
        btn1.begin(); btn2.begin();
        btn3.begin(); btnEnc.begin();
        encoder.attachHalfQuad(S1, S2);
    }

    void scan() {
        btn1.read(); btn2.read();
        btn3.read(); btnEnc.read();
        
        int event = 0;
        if(btn1.wasReleased()) event = 1;
        if(btn2.wasReleased()) event = 2;
        // ...其他事件检测
        
        xQueueSend(inputQueue, &event, 0);
    }
} inputHandler;

//======================================
// Class: DisplayManager
//======================================
class DisplayManager {
private:
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
    // 菜单状态变量...
    
public:
    DisplayManager() : u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA) {}

    void init() {
        u8g2.begin();
        u8g2.setFont(u8g2_font_ncenB08_tr);
    }

    void update(int event) {
        // 处理事件更新显示
        u8g2.clearBuffer();
        // 绘制菜单逻辑...
        u8g2.sendBuffer();
    }
} displayMgr;

//======================================
// FreeRTOS任务函数
//======================================
void motorTask(void *pv) {
    while(1) {
        motorCtrl.update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void inputTask(void *pv) {
    while(1) {
        inputHandler.scan();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void displayTask(void *pv) {
    int event;
    while(1) {
        if(xQueueReceive(inputQueue, &event, portMAX_DELAY)) {
            displayMgr.update(event);
        }
    }
}

//======================================
// 初始化函数
//======================================
void setup() {
    Serial.begin(9600);
    
    // 初始化硬件
    motorCtrl.init();
    inputHandler.init();
    displayMgr.init();

    // 创建FreeRTOS任务
    xTaskCreatePinnedToCore(motorTask, "Motor", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(inputTask, "Input", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(displayTask, "Display", 8192, NULL, 1, NULL, 1);

    // 删除默认的Arduino循环任务
    vTaskDelete(NULL);
}

void loop() {} // FreeRTOS接管后不需要
