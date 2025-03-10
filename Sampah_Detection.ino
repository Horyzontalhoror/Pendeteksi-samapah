#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp32-hal-ledc.h"
//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//this edit code doesn't use a gzipped html source code
//the html -code is open and is easy to update or modify  on the other tab page = app_httpd.cpp

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_M5STACK_PSRAM
#define CAMERA_MODEL_AI_THINKER.

#include "camera_pins.h"
#define TIMER_WIDTH 16

int trigPin = 13;   //Trig
int echoPin = 15;   //Echo

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

//void startCameraServer();
int deteksi();

void setup() {


  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  ledcSetup(3, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(12, 3); 
  ledcSetup(2, 50, TIMER_WIDTH);
  ledcAttachPin(3, 2);
  Serial.println();
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
  camera_config_t config;
//  config.ledc_channel = LEDC_CHANNEL_0;
//  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_240X240;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_240X240;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  //drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_240X240);

}

void loop() {
  // put your main code here, to run repeatedly:

  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  if (distance >= 10 && distance <= 30) {
    Serial.print("detek: ");
    Serial.print(deteksi());
    int detek = deteksi();
    if (detek == 0) {
      bukaKanan();
    }
    else {
      bukaKiri();
    }
//    delay(4000);
  }
  delay(1000);

//  Serial.print("detek: ");
//  Serial.print(deteksi());
//  int detek = deteksi();
//  if (detek == 0) {
//    bukaKanan();
//  }
//  else {
//    bukaKiri();
//  }
//  delay(4000);
}

void bukaKiri() {
  Serial.print("Buka Kiri");
  ledcWrite(3, 1464);
  ledcWrite(3, 5338);
  delay(3000);
  ledcWrite(3, 1464);
//  delay(10000);
}

void bukaKanan() {
  Serial.print("Buka Kanan");
  ledcWrite(2, 5538);
  ledcWrite(2, 1564);
  delay(3000);
  ledcWrite(2, 5538);
//  delay(10000);
}
