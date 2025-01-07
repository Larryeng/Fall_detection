#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <FastLED.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <math.h> //包含數學庫以使用 sqrt()
//===========================================================
const char *ssid = "Galaxy A52s 5G";  //Wi-Fi 名稱
const char *password = "Larry970411"; //Wi-Fi 密碼
const char *discordWebhook = "https://discord.com/api/webhooks/1308277921416351796/m_hdBxAEGPhGKQRgTEw7ZaxPKenkuq82VKNOhJPKwknnWlJF2LNgOGPYvhgehtX1wdrj"  //your Discord APi(This APi is unusable,Don't use it😒);
//加速度計
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
//標記變數設定
bool startAlarn = true;
bool startF = false; //自由落體開始標記
bool Fallen = false; //衝擊檢測標記
bool lying = false;  //靜止或平躺標記
//閾值設定
const float FAL_scale = 15; //自由落體閾值(加速度)
const float IMPAC_scale = 25;     //衝擊閾值 (加速度)
const float LYING_scale = 3;      //靜止狀態閾值 (加速度)
//const float LYING_scale_1 = 5;       //靜止容差範圍 (加速度)
//按鈕設定
const int BUTTON_PIN = 2;
//LED 設定
CRGB leds[1];
//===========================================================
//偵測重置
void resetDetection()
{
    startF = false;
    Fallen = false;
    lying = false;
    leds[0] = CRGB::Black;
    FastLED.show();
    digitalWrite(4, HIGH); //關閉蜂鳴器
    Serial.println("偵測狀態重置完成");
}
//警報觸發
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
//wifi
void connectToWiFi()
{
    Serial.print("正在連接 Wi-Fi");
    WiFi.begin(ssid, password);

    unsigned long startAttemptTime = millis();
    const unsigned long timeout = 10000; //設置超時時間為10秒

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout)
    {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWi-Fi 已連接");
        Serial.print("IP 地址: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWi-Fi 連接失敗，繼續執行程序");
    }
}
//===========================================================
void setup()
{
    Serial.begin(115200);
    connectToWiFi();
    //確認成功搜尋到感應器
    if (!accel.begin())
    {
        Serial.println("無法找到 ADXL345 感應器");
        while (1)
            ;
    }
    Serial.println("ADXL345 初始化完成");

    //設定範圍
    Serial.println("設定範圍 ±16G");
    accel.setRange(ADXL345_RANGE_16_G);
    //蜂鳴器初始
    pinMode(4, OUTPUT);
    digitalWrite(4, HIGH);
    //初始化LED
    FastLED.addLeds<WS2812, 1, RGB>(leds, 1);
    leds[0] = CRGB::Black; //設定 LED 顏色為黑色
    FastLED.show();

}
//===========================================================
void loop()
{
    sensors_event_t event;
    accel.getEvent(&event);
    float SV = (sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow((event.acceleration.z), 2)))-9.8;

    //輸出現在加速度範圍
    Serial.println("總加速度 :");
    Serial.println(SV);

    //自由落體檢測
    //1.檢測以跌倒自由落體
    if (SV < FAL_scale && SV>0 && !startF)
    {
        startF = true;
        Serial.println("跌倒開始");
    }
    //2.檢測是否已經落地
    if (SV > IMPAC_scale && SV>0 && startF)
    {
        Fallen = true;
        Serial.println("衝擊檢測 : 確認跌倒偵測已啟動");
        delay(100);
        
    }
//3.檢測是否已經靜止
    if (SV < LYING_scale && SV>0 && Fallen)
    {
        lying = true;
        Serial.println("躺平檢測 : 已躺平");
        delay(100);
        for (int i = 0; i < 20; i++) {
            if(digitalRead(BUTTON_PIN)== LOW){ 
                resetDetection();
                startAlarn = false;
                break;
            }
            FastLED.setBrightness(255);
            leds[0] = CRGB::Red; //設定 LED 顏色為紅色
            FastLED.show();
            digitalWrite(4, LOW);
            delay(500);
            leds[0] = CRGB::Black; //設定 LED 顏色為黑色
            FastLED.show();
            digitalWrite(4, HIGH);
            delay(500);
            if(digitalRead(BUTTON_PIN)== LOW){ 
                resetDetection();
                startAlarn = false;
                break;
            }
        }
        if(startAlarn){
            Alarm();
        }
    }
    delay(100);
}


