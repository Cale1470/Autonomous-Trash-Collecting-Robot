/* Edge Impulse Arduino examples */
#include <kim_belledo-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"

#define CAMERA_MODEL_AI_THINKER // Use AI Thinker ESP32-CAM

#if defined(CAMERA_MODEL_AI_THINKER)
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
#else
#error "Camera model not selected"
#endif

// Define UART pins for communication with Arduino
#define RXD2 16  // ESP32 RX2 pin (connect to Arduino TX)
#define TXD2 17  // ESP32 TX2 pin (connect to Arduino RX)

/* Constants */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

/* Variables */
static bool is_initialised = false;
static bool debug_nn = false;
uint8_t *snapshot_buf;
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function prototypes */
bool ei_camera_init(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);

void setup() {
    Serial.begin(115200);  // For debugging
    while (!Serial);
    Serial.println("ESP32-CAM Object Detection System");

    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // For Arduino communication
    delay(1000);  // Give some time for serial to stabilize

    if (!ei_camera_init()) {
        Serial.println("Camera failed to initialize!");
    } else {
        Serial.println("Camera initialized successfully.");
    }

    Serial.println("Starting inference in 2 seconds...");
    delay(2000);
}

void loop() {
    if (ei_sleep(5) != EI_IMPULSE_OK) return;

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if (!snapshot_buf) {
        Serial.println("Error: Failed to allocate snapshot buffer!");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
        Serial.println("Error: Failed to capture image.");
        free(snapshot_buf);
        return;
    }

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        Serial.printf("Error: Classifier failed (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    Serial.println("Predictions:");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        Serial.printf("  %s: %.5f\n", ei_classifier_inferencing_categories[i], result.classification[i].value);
    }

    // Object Detection Logic
#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value > 0.5) {  // If detection confidence > 50%
            Serial2.println("Object Detected"); // Send to Arduino
            Serial.printf("Object Detected: %s (%f) [x: %u, y: %u, width: %u, height: %u]\n",
                bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
        }
    }
#endif

    free(snapshot_buf);
}

bool ei_camera_init() {
    if (is_initialised) return true;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

    is_initialised = true;
    return true;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    if (!is_initialised) {
        Serial.println("Error: Camera is not initialized.");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Error: Camera capture failed.");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if (!converted) {
        Serial.println("Error: Conversion failed.");
        return false;
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left--) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) +
                              (snapshot_buf[pixel_ix + 1] << 8) +
                              snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
    }

    return 0;
}