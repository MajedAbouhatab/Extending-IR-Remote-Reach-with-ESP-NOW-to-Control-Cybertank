struct struct_message
{
    bool R;
    bool B;
    bool L;
    bool F;
};

struct_message DirData;
volatile unsigned long TimeStamp;

#if defined(ARDUINO_ESP8266_WEMOS_D1MINI)
#include <ESP8266WiFi.h>
#include <espnow.h>

#define IRPin D5

volatile unsigned long BitPattern = 0;
uint8_t ReceiverAddress[] = {0xE8, 0x6B, 0xEA, 0xDF, 0x04, 0x94};

void IRAM_ATTR handleSignalEdge()
{
    if (micros() - TimeStamp > 40)
        if (micros() - TimeStamp > 35000UL)
            BitPattern = 0;
        else
            BitPattern = (BitPattern << 1) | (micros() - TimeStamp < 1250);
    TimeStamp = micros();
}

void setup()
{
    Serial.begin(115200);
    pinMode(IRPin, INPUT_PULLUP);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    esp_now_init();
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(ReceiverAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    attachInterrupt(digitalPinToInterrupt(IRPin), handleSignalEdge, FALLING);
}

void loop()
{
    if (BitPattern & (1UL << 11))
    {
        DirData = struct_message{BitPattern == 0xD04, BitPattern == 0xD08, BitPattern == 0xD10, BitPattern == 0xD20};
        esp_now_send(ReceiverAddress, (uint8_t *)&DirData, sizeof(DirData));
        Serial.println(BitPattern, HEX);
        BitPattern = 0;
    }
    yield();
}

#elif defined(ARDUINO_ESP32_DEV)

#include <WiFi.h>
#include <esp_now.h>

// From https://github.com/adafruit/Adafruit_Motor-Shield-v1/blob/master/mshieldv12schem.png
// #define MOTOR2_PWM  D3
#define MOTORCLK D4
#define MOTOR3_PWM D5
#define MOTOR4_PWM D6
#define MOTORENABLE D7
#define MOTORDATA D8
// #define SERVO2_PIN  D9
// #define SERVO1_PIN  D10
// #define MOTOR1_PWM  D11
#define MOTORLATCH D12

uint8_t latch_state = 0, MotorMinMax;

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    TimeStamp = millis();
    memcpy(&DirData, data, sizeof(DirData));
    Serial.printf("%lu R=%d, B=%d, L=%d, F=%d\n", TimeStamp, DirData.R, DirData.B, DirData.L, DirData.F);
}

void MotorDir(uint8_t Motor, int Dir)
{
    bitWrite(latch_state, 5 * (3 - Motor), (Dir > 0) ? HIGH : LOW);
    bitWrite(latch_state, 7 - (Motor - 2), (Dir < 0) ? HIGH : LOW);
    digitalWrite(MOTORLATCH, LOW);
    shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_state);
    digitalWrite(MOTORLATCH, HIGH);
}

void motorTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (true)
    {
        if (millis() - TimeStamp > 200)
        {
            DirData = struct_message{0, 0, 0, 0};
            digitalWrite(MOTORENABLE, HIGH);
        }
        else if (DirData.R || DirData.B || DirData.L || DirData.F)
        {
            digitalWrite(MOTORENABLE, LOW);
            MotorMinMax = (DirData.L || DirData.R) ? 0xFF * .8 : 0xFF / 2;
            analogWrite(MOTOR3_PWM, MotorMinMax);
            analogWrite(MOTOR4_PWM, MotorMinMax);
            MotorDir(2, constrain(255 * (DirData.F - DirData.B + DirData.R - DirData.L), -1 * MotorMinMax, MotorMinMax));
            MotorDir(3, constrain(255 * (DirData.F - DirData.B - DirData.R + DirData.L), -1 * MotorMinMax, MotorMinMax));
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
    }
}

void setup(void)
{
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("uint8_t ReceiverAddress[] = {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X};\n\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_now_init();
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    pinMode(MOTORLATCH, OUTPUT);
    pinMode(MOTORCLK, OUTPUT);
    pinMode(MOTORDATA, OUTPUT);
    pinMode(MOTOR3_PWM, OUTPUT);
    pinMode(MOTOR4_PWM, OUTPUT);
    pinMode(MOTORENABLE, OUTPUT);
    digitalWrite(MOTORENABLE, HIGH);
    xTaskCreatePinnedToCore(motorTask, "MotorTask", 2048, NULL, 1, NULL, 1);
}

void loop(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000));
}
#else
#error Unsupported board selection.
#endif
