#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <FastLED.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <math.h> //Include the math library to use sqrt()
//===========================================================
const char *ssid = "Galaxy A52s 5G";  // Wi-Fi name
const char *password = "Larry970411"; // Wi-Fi password
const char *discordWebhook = "https://discord.com/api/webhooks/1308277921416351796/m_hdBxAEGPhGKQRgTEw7ZaxPKenkuq82VKNOhJPKwknnWlJF2LNgOGPYvhgehtX1wdrj";
// accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
// variable initialization
bool startAlarn = true;
bool startF = false; // Free fall start marker
bool Fallen = false; // Impact detection marker
bool lying = false;  // Rest or flat position marker
// Threshold setting
const float FAL_scale = 15;   // Free fall threshold (acceleration)
const float IMPAC_scale = 25; // Impact threshold (acceleration)
const float LYING_scale = 3;  // Rest state threshold (acceleration)
// const float LYING_scale_1 = 5;       //Rest tolerance range (acceleration)
// Button settings
const int BUTTON_PIN = 2;
// LED settings
CRGB leds[1];
//===========================================================
// LED settings
void resetDetection()
{
    startF = false;
    Fallen = false;
    lying = false;
    leds[0] = CRGB::Black;
    FastLED.show();
    digitalWrite(4, HIGH); // Turn off the buzzer
    Serial.println("偵測狀態重置完成");
}
// Trigger alarm
void Alarm()
{
    Serial.println("警報：跌倒事件確認！");

    if (WiFi.status() == WL_CONNECTED)
    {
        HTTPClient http;
        http.begin(discordWebhook);
        http.addHeader("Content-Type", "application/json");

        String jsonPayload = R"(
    {
      "username": "Fall Detection Bot",
      "embeds": [
        {
            "title": "⚠️ **跌倒警報觸發！** 請立即查看配戴者狀況。",
            "color": null,
            "color": null,
            "image": {
                "url": "https://i.imgur.com/pTOOLNc.gif"
        }
        }
    ],
      "avatar_url": ""
    }
    )";

        int httpResponseCode = http.POST(jsonPayload);
        if (httpResponseCode > 0)
        {
            Serial.printf("Discord Webhook 傳送成功，回應碼: %d\n", httpResponseCode);
        }
        else
        {
            Serial.printf("Webhook 傳送失敗，錯誤碼: %d\n", httpResponseCode);
        }
        http.end();
    }
    else
    {
        Serial.println("Wi-Fi 未連線，無法傳送警報！");
    }
    resetDetection();
}
//===========================================================
//===========================================================
// wifi
void connectToWiFi()
{
    Serial.print("正在連接 Wi-Fi");
    WiFi.begin(ssid, password);

    unsigned long startAttemptTime = millis();
    const unsigned long timeout = 10000; // Set timeout to 10 seconds

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout)
    {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWi-Fi 已連接");
        Serial.print("IP 地址: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("\nWi-Fi 連接失敗，繼續執行程序");
    }
}
//===========================================================
void setup()
{
    Serial.begin(115200);
    connectToWiFi();
    // Confirm successful sensor detection
    if (!accel.begin())
    {
        Serial.println("無法找到 ADXL345 感應器");
        while (1)
            ;
    }
    Serial.println("ADXL345 初始化完成");

    // Set range
    Serial.println("設定範圍 ±16G");
    accel.setRange(ADXL345_RANGE_16_G);
    // Buzzer initialization
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    // Initialize LED
    FastLED.addLeds<WS2812, 1, RGB>(leds, 1);
    leds[0] = CRGB::Black; // Set LED color to black
    FastLED.show();
}
//===========================================================
void loop()
{
    sensors_event_t event;
    accel.getEvent(&event);
    float SV = (sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow((event.acceleration.z), 2))) - 9.8;

    // Output current acceleration range
    Serial.println("總加速度 :");
    Serial.println(SV);

    // Free fall detection
    // 1.Detect free fall as a fall
    if (SV < FAL_scale && SV > 0 && !startF)
    {
        startF = true;
        Serial.println("跌倒開始");
    }
    // 2.Detect if it has landed
    if (SV > IMPAC_scale && SV > 0 && startF)
    {
        Fallen = true;
        Serial.println("衝擊檢測 : 確認跌倒偵測已啟動");
        delay(100);
    }
    // 3.Detect if it has stopped moving
    if (SV < LYING_scale && SV > 0 && Fallen)
    {
        lying = true;
        Serial.println("躺平檢測 : 已躺平");
        delay(100);
        for (int i = 0; i < 20; i++)
        {
            if (digitalRead(BUTTON_PIN) == LOW)
            {
                resetDetection();
                startAlarn = false;
                break;
            }
            FastLED.setBrightness(255);
            leds[0] = CRGB::Red; // Set LED color to red
            FastLED.show();
            digitalWrite(4, LOW);
            delay(500);
            leds[0] = CRGB::Black; // Set LED color to black
            FastLED.show();
            digitalWrite(4, HIGH);
            delay(500);
            if (digitalRead(BUTTON_PIN) == LOW)
            {
                resetDetection();
                startAlarn = false;
                break;
            }
        }
        if (startAlarn)
        {
            Alarm();
        }
    }
    delay(100);
}
