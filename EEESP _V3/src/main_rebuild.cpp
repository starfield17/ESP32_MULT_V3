#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <PID_v1.h>
#include <JC_Button.h>
#include <ezBuzzer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Debug mode
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
// Gyroscope
#define MPU_INT 8
#define MPU_SCL 17
#define MPU_SDA 18
// NRF
#define SET 15
#define BEEP 16
#define IRQ 3
#define MISO 46
#define MOSI 9
#define SCK 10
#define CSN 11
#define CE 12

// FreeRTOS task parameters
#define MOTOR_CONTROL_PRIORITY (tskIDLE_PRIORITY + 3)
#define UI_PRIORITY (tskIDLE_PRIORITY + 2)
#define INPUT_PRIORITY (tskIDLE_PRIORITY + 2)
#define SENSOR_PRIORITY (tskIDLE_PRIORITY + 1)
#define SOUND_PRIORITY (tskIDLE_PRIORITY + 1)

#define MOTOR_STACK_SIZE 2048
#define UI_STACK_SIZE 3072
#define INPUT_STACK_SIZE 2048
#define SENSOR_STACK_SIZE 2048
#define SOUND_STACK_SIZE 1024

// Forward declarations
class MotorController;
class InputHandler;
class DisplayManager;
class SensorManager;
class MenuSystem;

// ===== State definitions =====
// Menu system states
enum class MenuState {
    MAIN_MENU = -1,
    SPEED_SETTING = 0,
    GYRO_VIEW = 1,
    PID_SETTING = 2
};

// PID adjustment sub-states
enum class PIDState {
    NONE = 0,
    P_ADJUST = 1,
    I_ADJUST = 2,
    D_ADJUST = 3
};

// ===== Global device instances =====
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
Adafruit_MPU6050 mpu;
ezBuzzer buzzer(BEEP);

// ===== Motor control class =====
class MotorController {
private:
    ESP32Encoder motorEncoder1;
    ESP32Encoder motorEncoder2;
    
    double setpoint1, input1, output1;
    double setpoint2, input2, output2;
    double kp, ki, kd;
    
    PID pid1;
    PID pid2;
    
    int pwmFrequency;
    SemaphoreHandle_t paramMutex;

public:
    MotorController() : 
        setpoint1(1000), input1(0), output1(0),
        setpoint2(1000), input2(0), output2(0),
        kp(2.0), ki(5.0), kd(1.0),
        pid1(&input1, &output1, &setpoint1, kp, ki, kd, DIRECT),
        pid2(&input2, &output2, &setpoint2, kp, ki, kd, DIRECT),
        pwmFrequency(1000) {
        paramMutex = xSemaphoreCreateMutex();
    }
    
    void init() {
        // Initialize encoders
        ESP32Encoder::useInternalWeakPullResistors = UP;
        motorEncoder1.attachHalfQuad(INA1, INA2);
        motorEncoder1.clearCount();
        motorEncoder2.attachHalfQuad(INB1, INB2);
        motorEncoder2.clearCount();
        
        // Initialize PWM
        ledcSetup(0, pwmFrequency, 8); // AIN1
        ledcSetup(1, pwmFrequency, 8); // AIN2
        ledcSetup(2, pwmFrequency, 8); // BIN1
        ledcSetup(3, pwmFrequency, 8); // BIN2
        
        ledcAttachPin(AIN1, 0);
        ledcAttachPin(AIN2, 1);
        ledcAttachPin(BIN1, 2);
        ledcAttachPin(BIN2, 3);
        
        // Initialize PID
        pid1.SetMode(AUTOMATIC);
        pid2.SetMode(AUTOMATIC);
    }
    
    void update() {
        xSemaphoreTake(paramMutex, portMAX_DELAY);
        
        input1 = motorEncoder1.getCount();
        input2 = motorEncoder2.getCount();
        
        pid1.Compute();
        pid2.Compute();
        
        #if MOTOR_ON
        ledcWrite(0, output1); // AIN1
        ledcWrite(2, output2); // BIN1
        #endif
        
        xSemaphoreGive(paramMutex);
    }
    
    // Adjust speed setpoint
    void adjustSpeed(double delta) {
        xSemaphoreTake(paramMutex, portMAX_DELAY);
        setpoint1 += delta;
        setpoint2 += delta;
        xSemaphoreGive(paramMutex);
    }
    
    // Adjust PID parameters
    void adjustP(double delta) {
        xSemaphoreTake(paramMutex, portMAX_DELAY);
        kp += delta;
        pid1.SetTunings(kp, ki, kd);
        pid2.SetTunings(kp, ki, kd);
        xSemaphoreGive(paramMutex);
    }
    
    void adjustI(double delta) {
        xSemaphoreTake(paramMutex, portMAX_DELAY);
        ki += delta;
        pid1.SetTunings(kp, ki, kd);
        pid2.SetTunings(kp, ki, kd);
        xSemaphoreGive(paramMutex);
    }
    
    void adjustD(double delta) {
        xSemaphoreTake(paramMutex, portMAX_DELAY);
        kd += delta;
        pid1.SetTunings(kp, ki, kd);
        pid2.SetTunings(kp, ki, kd);
        xSemaphoreGive(paramMutex);
    }
    
    // Get current values
    double getSetpoint1() {
        double value;
        xSemaphoreTake(paramMutex, portMAX_DELAY);
        value = setpoint1;
        xSemaphoreGive(paramMutex);
        return value;
    }
    
    double getSetpoint2() {
        double value;
        xSemaphoreTake(paramMutex, portMAX_DELAY);
        value = setpoint2;
        xSemaphoreGive(paramMutex);
        return value;
    }
    
    double getP() {
        double value;
        xSemaphoreTake(paramMutex, portMAX_DELAY);
        value = kp;
        xSemaphoreGive(paramMutex);
        return value;
    }
    
    double getI() {
        double value;
        xSemaphoreTake(paramMutex, portMAX_DELAY);
        value = ki;
        xSemaphoreGive(paramMutex);
        return value;
    }
    
    double getD() {
        double value;
        xSemaphoreTake(paramMutex, portMAX_DELAY);
        value = kd;
        xSemaphoreGive(paramMutex);
        return value;
    }
};

// ===== Input handling class =====
class InputHandler {
private:
    ESP32Encoder encoder;
    Button button1;
    Button button2;
    Button button3;
    Button encoderButton;
    
    int64_t position;
    
    struct InputState {
        bool b1Pressed;
        bool b2Pressed;
        bool b3Pressed;
        bool encPressed;
        bool leftTurn;
        bool rightTurn;
    } state;
    
    SemaphoreHandle_t inputMutex;

public:
    InputHandler() :
        button1(BUTTON1),
        button2(BUTTON2),
        button3(BUTTON3),
        encoderButton(ENCODER_KEY),
        position(0) {
        
        memset(&state, 0, sizeof(state));
        inputMutex = xSemaphoreCreateMutex();
    }
    
    void init() {
        button1.begin();
        button2.begin();
        button3.begin();
        encoderButton.begin();
        
        ESP32Encoder::useInternalWeakPullResistors = UP;
        encoder.attachHalfQuad(S1, S2);
        encoder.clearCount();
    }
    
    void scan() {
        button1.read();
        button2.read();
        button3.read();
        encoderButton.read();
        
        xSemaphoreTake(inputMutex, portMAX_DELAY);
        
        // Detect button presses
        if (button1.wasReleased()) {
            state.b1Pressed = true;
        }
        
        if (button2.wasReleased()) {
            state.b2Pressed = true;
        }
        
        if (button3.wasReleased()) {
            state.b3Pressed = true;
        }
        
        if (encoderButton.wasReleased()) {
            state.encPressed = true;
        }
        
        // Detect encoder rotation
        int64_t newPosition = encoder.getCount();
        if (newPosition > position) {
            state.rightTurn = true;
            #if DEBUG
            Serial.println("Clockwise");
            #endif
        } else if (newPosition < position) {
            state.leftTurn = true;
            #if DEBUG
            Serial.println("Counterclockwise");
            #endif
        }
        
        position = newPosition;
        
        xSemaphoreGive(inputMutex);
    }
    
    // Get and clear input state
    bool getAndClearB1() {
        bool result;
        xSemaphoreTake(inputMutex, portMAX_DELAY);
        result = state.b1Pressed;
        state.b1Pressed = false;
        xSemaphoreGive(inputMutex);
        return result;
    }
    
    bool getAndClearB2() {
        bool result;
        xSemaphoreTake(inputMutex, portMAX_DELAY);
        result = state.b2Pressed;
        state.b2Pressed = false;
        xSemaphoreGive(inputMutex);
        return result;
    }
    
    bool getAndClearB3() {
        bool result;
        xSemaphoreTake(inputMutex, portMAX_DELAY);
        result = state.b3Pressed;
        state.b3Pressed = false;
        xSemaphoreGive(inputMutex);
        return result;
    }
    
    bool getAndClearEnc() {
        bool result;
        xSemaphoreTake(inputMutex, portMAX_DELAY);
        result = state.encPressed;
        state.encPressed = false;
        xSemaphoreGive(inputMutex);
        return result;
    }
    
    bool getAndClearLeft() {
        bool result;
        xSemaphoreTake(inputMutex, portMAX_DELAY);
        result = state.leftTurn;
        state.leftTurn = false;
        xSemaphoreGive(inputMutex);
        return result;
    }
    
    bool getAndClearRight() {
        bool result;
        xSemaphoreTake(inputMutex, portMAX_DELAY);
        result = state.rightTurn;
        state.rightTurn = false;
        xSemaphoreGive(inputMutex);
        return result;
    }
};

// ===== Sensor management class =====
class SensorManager {
private:
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    SemaphoreHandle_t sensorMutex;

public:
    SensorManager() {
        sensorMutex = xSemaphoreCreateMutex();
    }
    
    bool init() {
        Wire1.begin(MPU_SDA, MPU_SCL);
        Wire1.setClock(400000); // Set I2C frequency to 400kHz
        
        if (!mpu.begin(0x68, &Wire1)) {
            Serial.println("Failed to find MPU6050 chip");
            return false;
        }
        
        Serial.println("MPU6050 Found!");
        
        mpu.setSampleRateDivisor(19); // Sample rate = 1kHz / (1 + 19) = 50Hz
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        
        return true;
    }
    
    void update() {
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        mpu.getEvent(&accel, &gyro, &temp);
        xSemaphoreGive(sensorMutex);
    }
    
    float getGyroX() {
        float value;
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        value = gyro.gyro.x;
        xSemaphoreGive(sensorMutex);
        return value;
    }
    
    float getGyroY() {
        float value;
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        value = gyro.gyro.y;
        xSemaphoreGive(sensorMutex);
        return value;
    }
    
    float getGyroZ() {
        float value;
        xSemaphoreTake(sensorMutex, portMAX_DELAY);
        value = gyro.gyro.z;
        xSemaphoreGive(sensorMutex);
        return value;
    }
};

// ===== Buzzer management class =====
class SoundManager {
public:
    void init() {
        // Buzzer already initialized in global variable
    }
    
    void beep(int duration) {
        buzzer.beep(duration);
    }
    
    void update() {
        buzzer.loop();
        
        // Process serial commands
        if (Serial.available() > 0) {
            char incomingByte = Serial.read();
            if (incomingByte == '1') {
                beep(500);
            }
        }
    }
};

// ===== Menu system class =====
class MenuSystem {
private:
    struct MenuItem {
        int number;
        MenuItem* next;
        MenuItem* prev;
    };
    
    MenuItem item0, item1, item2;
    MenuItem* currentItem;
    
    MenuState currentState;
    PIDState pidState;
    
    MotorController& motors;
    InputHandler& input;
    SensorManager& sensors;
    SoundManager& sound;
    
    SemaphoreHandle_t menuMutex;

public:
    MenuSystem(MotorController& m, InputHandler& i, SensorManager& s, SoundManager& so) :
        motors(m), input(i), sensors(s), sound(so),
        currentState(MenuState::MAIN_MENU),
        pidState(PIDState::NONE) {
        
        // Initialize menu circular linked list
        item0 = {0, &item1, &item2};
        item1 = {1, &item2, &item0};
        item2 = {2, &item0, &item1};
        
        currentItem = &item0;
        menuMutex = xSemaphoreCreateMutex();
    }
    
    void update() {
        xSemaphoreTake(menuMutex, portMAX_DELAY);
        
        bool b1 = input.getAndClearB1();
        bool b2 = input.getAndClearB2();
        bool b3 = input.getAndClearB3();
        bool enc = input.getAndClearEnc();
        bool left = input.getAndClearLeft();
        bool right = input.getAndClearRight();
        
        // Handle state machine transitions and inputs
        switch (static_cast<int>(currentState)) {
            case static_cast<int>(MenuState::MAIN_MENU):
                handleMainMenu(b1, b2, b3, enc, left, right);
                break;
                
            case static_cast<int>(MenuState::SPEED_SETTING):
                handleSpeedSetting(b1, b2, b3, enc, left, right);
                break;
                
            case static_cast<int>(MenuState::GYRO_VIEW):
                handleGyroView(b1, b2, b3, enc, left, right);
                break;
                
            case static_cast<int>(MenuState::PID_SETTING):
                handlePIDSetting(b1, b2, b3, enc, left, right);
                break;
        }
        
        xSemaphoreGive(menuMutex);
    }
    
    // Draw current menu state
    void draw() {
        u8g2.clearBuffer();
        
        switch (static_cast<int>(currentState)) {
            case static_cast<int>(MenuState::MAIN_MENU):
                drawMainMenu();
                break;
                
            case static_cast<int>(MenuState::SPEED_SETTING):
                drawSpeedSetting();
                break;
                
            case static_cast<int>(MenuState::GYRO_VIEW):
                drawGyroView();
                break;
                
            case static_cast<int>(MenuState::PID_SETTING):
                drawPIDSetting();
                break;
        }
        
        u8g2.sendBuffer();
    }
    
private:
    // Main menu handling
    void handleMainMenu(bool b1, bool b2, bool b3, bool enc, bool left, bool right) {
        if (b1 || right) {
            currentItem = currentItem->next;
            u8g2.clearBuffer();
        }
        
        if (b3 || left) {
            currentItem = currentItem->prev;
            u8g2.clearBuffer();
        }
        
        if (b2) {
            switch (currentItem->number) {
                case 0:
                    currentState = MenuState::SPEED_SETTING;
                    break;
                case 1:
                    currentState = MenuState::GYRO_VIEW;
                    break;
                case 2:
                    currentState = MenuState::PID_SETTING;
                    pidState = PIDState::NONE;
                    break;
            }
            u8g2.clearBuffer();
        }
    }
    
    // Speed setting handling
    void handleSpeedSetting(bool b1, bool b2, bool b3, bool enc, bool left, bool right) {
        if (left) {
            motors.adjustSpeed(-50);
            u8g2.clearBuffer();
        }
        
        if (right) {
            motors.adjustSpeed(50);
            u8g2.clearBuffer();
        }
        
        if (enc) {
            currentState = MenuState::MAIN_MENU;
            u8g2.clearBuffer();
        }
    }
    
    // Gyroscope view handling
    void handleGyroView(bool b1, bool b2, bool b3, bool enc, bool left, bool right) {
        if (enc) {
            currentState = MenuState::MAIN_MENU;
            u8g2.clearBuffer();
        }
    }
    
    // PID setting handling
    void handlePIDSetting(bool b1, bool b2, bool b3, bool enc, bool left, bool right) {
        if (static_cast<int>(pidState) == static_cast<int>(PIDState::NONE)) {
            // Main PID menu
            if (b1 || right) {
                currentItem = currentItem->next;
                u8g2.clearBuffer();
            }
            
            if (b3 || left) {
                currentItem = currentItem->prev;
                u8g2.clearBuffer();
            }
            
            if (b2) {
                switch (currentItem->number) {
                    case 0:
                        pidState = PIDState::P_ADJUST;
                        break;
                    case 1:
                        pidState = PIDState::I_ADJUST;
                        break;
                    case 2:
                        pidState = PIDState::D_ADJUST;
                        break;
                }
                u8g2.clearBuffer();
            }
            
            if (enc) {
                currentState = MenuState::MAIN_MENU;
                u8g2.clearBuffer();
            }
        } else {
            // PID parameter adjustment
            if (left) {
                switch (static_cast<int>(pidState)) {
                    case static_cast<int>(PIDState::P_ADJUST):
                        motors.adjustP(-0.1);
                        break;
                    case static_cast<int>(PIDState::I_ADJUST):
                        motors.adjustI(-0.1);
                        break;
                    case static_cast<int>(PIDState::D_ADJUST):
                        motors.adjustD(-0.1);
                        break;
                }
                u8g2.clearBuffer();
            }
            
            if (right) {
                switch (static_cast<int>(pidState)) {
                    case static_cast<int>(PIDState::P_ADJUST):
                        motors.adjustP(0.1);
                        break;
                    case static_cast<int>(PIDState::I_ADJUST):
                        motors.adjustI(0.1);
                        break;
                    case static_cast<int>(PIDState::D_ADJUST):
                        motors.adjustD(0.1);
                        break;
                }
                u8g2.clearBuffer();
            }
            
            if (enc) {
                pidState = PIDState::NONE;
                u8g2.clearBuffer();
            }
        }
    }
    
    // Draw main menu
    void drawMainMenu() {
        u8g2.setFont(u8g2_font_ncenB08_tr);
        
        u8g2.drawStr(0, 10 + 2 * currentItem->number * u8g2.getFontAscent(), ">>");
        u8g2.drawStr(20, 10 + 2 * 0 * u8g2.getFontAscent(), "set Speed");
        u8g2.drawStr(20, 10 + 2 * 1 * u8g2.getFontAscent(), "LOOK XYZ");
        u8g2.drawStr(20, 10 + 2 * 2 * u8g2.getFontAscent(), "set PID");
    }
    
    // Draw speed setting interface
    void drawSpeedSetting() {
        u8g2.setFont(u8g2_font_ncenB08_tr);
        
        u8g2.drawStr(0, 10 + 0 * u8g2.getFontAscent(), "CHANGE_the_Speed");
        u8g2.drawStr(0, 10 + 2 * u8g2.getFontAscent(), "S1=");
        u8g2.drawStr(0, 10 + 4 * u8g2.getFontAscent(), "S2=");
        
        u8g2.drawStr(30, 10 + 2 * u8g2.getFontAscent(), String(motors.getSetpoint1()).c_str());
        u8g2.drawStr(30, 10 + 4 * u8g2.getFontAscent(), String(motors.getSetpoint2()).c_str());
    }
    
    // Draw gyroscope data
    void drawGyroView() {
        u8g2.setFont(u8g2_font_ncenB08_tr);
        
        u8g2.drawStr(20, 10 + 0 * u8g2.getFontAscent(), "X=");
        u8g2.drawStr(20, 10 + 2 * u8g2.getFontAscent(), "Y=");
        u8g2.drawStr(20, 10 + 4 * u8g2.getFontAscent(), "Z=");
        
        u8g2.drawStr(40, 10 + 0 * u8g2.getFontAscent(), String(sensors.getGyroX()).c_str());
        u8g2.drawStr(40, 10 + 2 * u8g2.getFontAscent(), String(sensors.getGyroY()).c_str());
        u8g2.drawStr(40, 10 + 4 * u8g2.getFontAscent(), String(sensors.getGyroZ()).c_str());
    }
    
    // Draw PID setting interface
    void drawPIDSetting() {
        u8g2.setFont(u8g2_font_ncenB08_tr);
        
        if (static_cast<int>(pidState) == static_cast<int>(PIDState::NONE)) {
            u8g2.drawStr(0, 10 + 2 * currentItem->number * u8g2.getFontAscent(), ">>");
        }
        
        u8g2.drawStr(20, 10 + 0 * u8g2.getFontAscent(), "P=");
        u8g2.drawStr(20, 10 + 2 * u8g2.getFontAscent(), "I=");
        u8g2.drawStr(20, 10 + 4 * u8g2.getFontAscent(), "D=");
        
        u8g2.drawStr(40, 10 + 0 * u8g2.getFontAscent(), String(motors.getP()).c_str());
        u8g2.drawStr(40, 10 + 2 * u8g2.getFontAscent(), String(motors.getI()).c_str());
        u8g2.drawStr(40, 10 + 4 * u8g2.getFontAscent(), String(motors.getD()).c_str());
    }
};

// ===== Integrated system class =====
class RobotSystem {
private:
    MotorController motors;
    InputHandler input;
    SensorManager sensors;
    SoundManager sound;
    MenuSystem menu;
    
    // FreeRTOS task handles
    TaskHandle_t motorTaskHandle;
    TaskHandle_t displayTaskHandle;
    TaskHandle_t inputTaskHandle;
    TaskHandle_t sensorTaskHandle;
    TaskHandle_t soundTaskHandle;
    
    // Task functions
    static void motorTask(void* parameter) {
        RobotSystem* system = (RobotSystem*)parameter;
        const TickType_t xDelay = pdMS_TO_TICKS(50);
        
        for (;;) {
            system->motors.update();
            vTaskDelay(xDelay);
        }
    }
    
    static void displayTask(void* parameter) {
        RobotSystem* system = (RobotSystem*)parameter;
        const TickType_t xDelay = pdMS_TO_TICKS(80);
        
        for (;;) {
            system->menu.update();
            system->menu.draw();
            vTaskDelay(xDelay);
        }
    }
    
    static void inputTask(void* parameter) {
        RobotSystem* system = (RobotSystem*)parameter;
        const TickType_t xDelay = pdMS_TO_TICKS(100);
        
        for (;;) {
            system->input.scan();
            vTaskDelay(xDelay);
        }
    }
    
    static void sensorTask(void* parameter) {
        RobotSystem* system = (RobotSystem*)parameter;
        const TickType_t xDelay = pdMS_TO_TICKS(40);
        
        for (;;) {
            system->sensors.update();
            vTaskDelay(xDelay);
        }
    }
    
    static void soundTask(void* parameter) {
        RobotSystem* system = (RobotSystem*)parameter;
        const TickType_t xDelay = pdMS_TO_TICKS(40);
        
        for (;;) {
            system->sound.update();
            vTaskDelay(xDelay);
        }
    }

public:
    RobotSystem() : menu(motors, input, sensors, sound) {}
    
    void init() {
        Serial.begin(9600);
        
        // Initialize all modules
        input.init();
        motors.init();
        
        if (!sensors.init()) {
            Serial.println("Sensor initialization failed!");
        }
        
        sound.init();
        u8g2.begin();
        
        // Create FreeRTOS tasks
        xTaskCreate(
            motorTask,
            "MotorTask",
            MOTOR_STACK_SIZE,
            this,
            MOTOR_CONTROL_PRIORITY,
            &motorTaskHandle
        );
        
        xTaskCreate(
            displayTask,
            "DisplayTask",
            UI_STACK_SIZE,
            this,
            UI_PRIORITY,
            &displayTaskHandle
        );
        
        xTaskCreate(
            inputTask,
            "InputTask",
            INPUT_STACK_SIZE,
            this,
            INPUT_PRIORITY,
            &inputTaskHandle
        );
        
        xTaskCreate(
            sensorTask,
            "SensorTask",
            SENSOR_STACK_SIZE,
            this,
            SENSOR_PRIORITY,
            &sensorTaskHandle
        );
        
        xTaskCreate(
            soundTask,
            "SoundTask",
            SOUND_STACK_SIZE,
            this,
            SOUND_PRIORITY,
            &soundTaskHandle
        );
    }
};

// Global system instance
RobotSystem robotSystem;

void setup() {
    robotSystem.init();
}

void loop() {
    // Empty loop, all functionality executed by FreeRTOS tasks
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
