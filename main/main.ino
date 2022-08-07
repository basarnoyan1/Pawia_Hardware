/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card

  IMPORTANT!!!
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include "esp_camera.h"
#include "CameraPins.h"
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <sim800c.h>

#define GSM_ENABLE_PIN GPIO_NUM_16
#define GSM_RX_PIN GPIO_NUM_15
#define GSM_TX_PIN GPIO_NUM_14
#define SCL_PIN GPIO_NUM_12
#define SDA_PIN GPIO_NUM_13
#define CAM_LED_PIN GPIO_NUM_4

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

MPU6050 mpu6050(Wire);
SIM800C gsmModule(GSM_RX_PIN, GSM_TX_PIN, GSM_ENABLE_PIN, 9600);

int pictureNumber = 0;

struct datas
{
    float x;
    float y;
    float z;
    float temperature;
    float LON;
    float LAT;
};

void setup()
{
    // cam setup
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
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
    if (psramFound())
    {
        config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
        config.jpeg_quality = 10;
        config.fb_count = 2;
    }
    else
    {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }
    esp_camera_init(&config);
    camera_fb_t *fb = NULL;
    // pin config
    pinMode(GSM_ENABLE_PIN, OUTPUT);
    pinMode(CAM_LED_PIN, OUTPUT);
    digitalWrite(CAM_LED_PIN, LOW);
    digitalWrite(GSM_ENABLE_PIN, HIGH);
    // MPU6050 Setup
    Wire.begin(SCL_PIN, SDA_PIN);
    mpu6050.begin();
    mpu6050.update();
    // GSM setup
    gsmModule.begin();
    // acquire data
    datas data;
    data.x = mpu6050.getAccelX();
    data.y = mpu6050.getAccelY();
    data.z = mpu6050.getAccelZ();
    data.temperature = mpu6050.getTemperature();
    data.LON = gsmModule.getLON();
    data.LAT = gsmModule.getLAT();
    fb = esp_camera_fb_get();
    //send data
    gsmModule.sendMQTT(tempBroker, data);
    gsmModule.sendFile(TempServer, fb);

    //sleep
    gsmModule.sleep();
    digitalWrite(CAM_LED_PIN, LOW);
    digitalWrite(GSM_ENABLE_PIN, LOW);
    esp_camera_deinit();
    mpuu6050.sleep();
    esp_sleep_enable_timer_wakeup(60 * 1000000);
}

void loop()
{
}
