// Include necessary libraries
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "quirc.h"
#include <set>

// UART library for ESP32
#include "HardwareSerial.h"

// Creating a task handle
TaskHandle_t QRCodeReader_Task;

// Define camera model
#define CAMERA_MODEL_AI_THINKER

// GPIO configuration for camera model AI Thinker
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

// Structure to hold QR code data
struct QRCodeData {
  bool valid;
  int dataType;
  uint8_t payload[1024];
  int payloadLen;
};

struct quirc *q = NULL;
uint8_t *image = NULL;  
camera_fb_t * fb = NULL;
struct quirc_code code;
struct quirc_data data;
quirc_decode_error_t err;
struct QRCodeData qrCodeData;  
String QRCodeResult = "";

// Global set to store scanned QR codes
std::set<String> scannedQRCodes;

// UART configuration
#define UART_NUM        0 // UART port number
#define BAUD_RATE       115200      // Baud rate for UART communication
#define TX_PIN          1         // TX pin GPIO number (GPIO 1)
#define RX_PIN          3         // RX pin GPIO number (GPIO 3)

// Initialize UART instance
HardwareSerial UART(UART_NUM);

// Function declarations
void QRCodeReader(void * pvParameters);
void dumpData(const struct quirc_data *data);

// Setup function
void setup() {
  pinMode(33, OUTPUT);

  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Init serial communication
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Initialize UART communication
  UART.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  // Camera configuration
  Serial.println("Start configuring and initializing the camera...");
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
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

  Serial.println("Configure and initialize the camera successfully.");
  Serial.println();

  // Create "QRCodeReader_Task"
  xTaskCreatePinnedToCore(
             QRCodeReader,          /* Task function */
             "QRCodeReader_Task",   /* name of task */
             10000,                 /* Stack size of task */
             NULL,                  /* parameter of the task */
             1,                     /* priority of the task */
             &QRCodeReader_Task,    /* Task handle to keep track of created task */
             0);                    /* pin task to core 0 */
}

// Loop function
void loop() {
  digitalWrite(33, LOW);
  delay(1);
}

// QRCodeReader task function
void QRCodeReader(void * pvParameters) {
  Serial.println("QRCodeReader is ready.");
  Serial.print("QRCodeReader running on core ");
  Serial.println(xPortGetCoreID());
  Serial.println();

  while (1) {
    q = quirc_new();
    if (q == NULL) {
      Serial.print("can't create quirc object\r\n");  
      continue;
    }

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      continue;
    }

    quirc_resize(q, fb->width, fb->height);
    image = quirc_begin(q, NULL, NULL);
    memcpy(image, fb->buf, fb->len);
    quirc_end(q);

    int count = quirc_count(q);
    if (count > 0) {
      quirc_extract(q, 0, &code);
      err = quirc_decode(&code, &data);

      if (err) {
        Serial.println("Decoding FAILED");
        QRCodeResult = "Decoding FAILED";
      } else {
        String result = (const char *)data.payload;
        if (scannedQRCodes.find(result) == scannedQRCodes.end()) {
          // New QR code, process it
          Serial.printf("Decoding successful:\n");
          dumpData(&data);
          scannedQRCodes.insert(result);
          
          // Send data via UART
          UART.println(result);
          Serial.println("Data sent through UART successfully");
        } else {
          Serial.println("QR code already scanned");
        }
      }
      Serial.println();
    }

    esp_camera_fb_return(fb);
    fb = NULL;
    image = NULL;  
    quirc_destroy(q);
  }
}

// Function to display the results of reading the QR Code on the serial monitor
void dumpData(const struct quirc_data *data) {
  Serial.printf("Version: %d\n", data->version);
  Serial.printf("ECC level: %c\n", "MLHQ"[data->ecc_level]);
  Serial.printf("Mask: %d\n", data->mask);
  Serial.printf("Length: %d\n", data->payload_len);
  Serial.printf("Payload: %s\n", data->payload);

  QRCodeResult = (const char *)data->payload;
}
