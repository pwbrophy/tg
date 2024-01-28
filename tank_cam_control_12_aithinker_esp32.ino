
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 02 ESP32 CAM Pan and Tilt Control SM (Station Mode)
/*
 * Reference :
 * - ESP32-CAM Pan and Tilt Video Streaming Web Server (2 Axis) : https://randomnerdtutorials.com/esp32-cam-pan-and-tilt-2-axis/
 * - ESP32Servo (Author: Kevin Harrington,John K. Bennett) : - https://www.arduino.cc/reference/en/libraries/esp32servo/
 *                                                           - https://github.com/madhephaestus/ESP32Servo
 * - Timer on ESP32 : https://openlabpro.com/guide/timer-on-esp32/
 * - HTML DOM TouchEvent : https://www.w3schools.com/jsref/obj_touchevent.asp
 * - HTML DOM MouseEvent : https://www.w3schools.com/jsref/obj_mouseevent.asp
 * - Add Space Between Buttons in HTML : https://www.delftstack.com/howto/html/html-space-between-buttons/
 * - CSS Buttons : https://www.w3schools.com/css/css3_buttons.asp
 * - How TO - Range Sliders : https://www.w3schools.com/howto/howto_js_rangeslider.asp
 * 
 * and from various other sources.
 */

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Important Information.                                                                                                                                                           //
// ----------------------                                                                                                                                                           //
// Since this project uses 2 servos, sufficient power supply is needed to drive 2 servos.                                                                                           //
// If the power supply is not enough, some errors will occur, such as ESP32 Cam restarts continuously, camera fails to capture or video stream cannot be loaded,                    //
// ESP32 Cam restarts when servo moves, video stream results streaky and others.                                                                                                    //
//                                                                                                                                                                                  //
// I have tested this project using a 5V 2A power supply and the result the project runs fine.                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// I realized that the HTML code in this project could have been better or simpler. But because of my limited knowledge on WEB programming,                                         //
// then that code is the best code I can make. If you can make it better or simpler, I hope you can share it with me or with the video viewers of this project.                     //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* ======================================== Including the libraries. */
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include <ESP32Servo.h>
/* ======================================== */

/* ======================================== Select camera model */
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM
#define CAMERA_MODEL_AI_THINKER
/* ======================================== */

/* ======================================== GPIO of camera models */
#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 25
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 32
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif defined(CAMERA_MODEL_M5STACK_PSRAM_B)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 22
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 32
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM 15
#define XCLK_GPIO_NUM 27
#define SIOD_GPIO_NUM 25
#define SIOC_GPIO_NUM 23

#define Y9_GPIO_NUM 19
#define Y8_GPIO_NUM 36
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 39
#define Y5_GPIO_NUM 5
#define Y4_GPIO_NUM 34
#define Y3_GPIO_NUM 35
#define Y2_GPIO_NUM 17
#define VSYNC_GPIO_NUM 22
#define HREF_GPIO_NUM 26
#define PCLK_GPIO_NUM 21

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Camera model not selected"
#endif
/* ======================================== */

// LEDs GPIO
#define LED_OnBoard 2

// SERVOs GPIO
#define servo_right_Pin 14   //green
#define servo_left_Pin 15    //orange
#define servo_turret_Pin 13  //yellow

/* ======================================== Variables declaration */
// setting PWM properties
const int PWM_freq = 5000;
const int ledChannel = 5;
const int PWM_resolution = 8;

// Constants for continuous servo control
const int STOP = 1500;                  // Stop pulse width in microseconds
const int SPEED_REVERSE = 500;          // Change in speed for each command
const int SPEED_FORWARDS = 500;         // Change in speed for each command
const float MAX_SPEED_FORWARD = 2000;   // Maximum speed in one direction
const float MAX_SPEED_BACKWARD = 1000;  // Maximum speed in opposite direction

bool acceleratingForwards = false;
bool acceleratingBackwards = false;
bool acceleratingLeft = false;
bool acceleratingRight = false;

float current_speed_left_f = STOP;   // floating-point current speed left
float current_speed_right_f = STOP;  // floating-point current speed right

const float ACCELERATION = 0.3;
const float DRAG = 0.3;

const float TURRET_ACCELERATION = 0.15;          // Speed for turret movement


float turret_current_speed = 1500;  // Starting position (middle)
float turret_max_speed = 1600;  // Starting position (middle)
float turret_min_speed = 1300;  // Starting position (middle)

bool turret_moving = false;         // Flag to indicate turret movement
bool turretMovingLeft = false;
bool turretMovingRight = false;

// Variable for servo position
int servo_right_Pos = 1500;
int servo_left_Pos = 1500;
/* ======================================== */

/* ======================================== Initialize servos */
/*
 * Maybe in this section you are a little confused, why did I initialize 4 servos, even though in this project only 2 servos are used. Below is a brief explanation.
 * 
 * The ESP32 has 4 hardware timers (please correct me if I'm wrong). If only initialize 2 servos, then automatically based on servo library, 
 * then servo_1 will use timer_0 (channel_0) and servo_2 will use timer_1 (channel_1). 
 * That would be a problem because timer_0 (channel_0) and timer_1 (channel_1) are already in use by the camera.
 * 
 * Therefore, initialization of 4 servos is used, so that the 2 servos that are actually used will shift to timer_2 (channel_2) and timer_3 (channel_3). Below is the simple flow:
 * 
 * servo_Dummy_1    --> timer_0 or channel_0
 * servo_Dummy_2    --> timer_1 or channel_1
 * servo_right  --> timer_2 or channel_2
 * servo_left     --> timer_3 or channel_3
 * 
 * Based on the flow above, the initialization of servo_Dummy_1 and servo_Dummy_2 is only used to shift the timer, 
 * so the 2 servos used in this project will use timer_2 (channel_2) and timer_3 (channel_3). 
 * servo_Dummy_1 and servo_Dummy_2 are not actually used, because timer_0 (channel_0) and timer_1 (channel_1) are already used by the camera.
 */
Servo servo_Dummy_1;
Servo servo_Dummy_2;
Servo servo_turret;
Servo servo_right;
Servo servo_left;

/* ======================================== */

/* ======================================== Replace with your network credentials */
const char *ssid = "ssid_here";
const char *password = "password_here";
/* ======================================== */

/* ======================================== */
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
/* ======================================== */

/* ======================================== Empty handle to esp_http_server */
httpd_handle_t index_httpd = NULL;
httpd_handle_t stream_httpd = NULL;
/* ======================================== */

/* ======================================== HTML code for index / main page */
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
<head>
    <title>ESP32-CAM Pan and Tilt Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
    <style>
        body { font-family: Arial; text-align: center; margin: 0px auto; padding-top: 10px; }
        .slidecontainer { width: 100%; }
        .slider {
            -webkit-appearance: none; width: 50%; height: 10px; border-radius: 5px;
            background: #d3d3d3; outline: none; opacity: 0.7; -webkit-transition: .2s; transition: opacity .2s;
        }
        .slider:hover { opacity: 1; }
        .slider::-webkit-slider-thumb {
            -webkit-appearance: none; appearance: none; width: 20px; height: 20px;
            border-radius: 50%; background: #04AA6D; cursor: pointer;
        }
        .slider::-moz-range-thumb {
            width: 20px; height: 20px; border-radius: 50%; background: #04AA6D; cursor: pointer;
        }
        .button {
            display: inline-block; padding: 15px; font-size: 16px; cursor: pointer;
            text-align: center; text-decoration: none; outline: none; color: #fff;
            background-color: #4CAF50; border: none; border-radius: 15px;
            box-shadow: 0 6px #999; -webkit-touch-callout: none;
            -webkit-user-select: none; -khtml-user-select: none;
            -moz-user-select: none; -ms-user-select: none; user-select: none; width: 30%
        }
        .button:hover { background-color: #3e8e41 }
        .button:active {
            background-color: #3e8e41; box-shadow: 0 2px #666; transform: translateY(4px);
        }
        .space { width: 15%; height: auto; display: inline-block; }
        img { width: auto; max-width: 100%; height: auto; }
    </style>
</head>
<body>
    <h3>ESP32-CAM Pan and Tilt Control</h3>
    <img src="" id="vdstream">
    <br><br>
    <div class="slidecontainer">
        <span style="font-size:15;">LED Flash &nbsp;</span>
        <input type="range" min="0" max="20" value="0" class="slider" id="mySlider">
    </div>
    <br>
    <div>
        <button class="button" data-command="turret_left"><b>TURRET LEFT</b></button>
        <div class="space"></div>
        <button class="button" data-command="turret_right"><b>TURRET RIGHT</b></button>
        <br><br>
        <button class="button" data-command="forwards"><b>FORWARDS</b></button>
        <br><br>
        <button class="button" data-command="left"><b>LEFT</b></button>
        <div class="space"></div>
        <button class="button" data-command="right"><b>RIGHT</b></button>
        <br><br>
        <button class="button" data-command="backwards"><b>BACKWARDS</b></button>
    </div>
<script>
// Set the video stream source when the window loads
window.onload = function() {
    document.getElementById("vdstream").src = window.location.href.slice(0, -1) + ":81/stream";
    startStopCommandInterval();
};

// Function to send movement commands to the server
function send_cmd(cmd) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/action?go=" + cmd, true);
    xhr.send();
}

// Variable to track if a command is already being sent
var commandActive = false;

// Interval ID for sending stop command
var stopCommandInterval;

// Function to start sending the stop command repeatedly
function startStopCommandInterval() {
    stopCommandInterval = setInterval(function() {
        if (!commandActive) {
            send_cmd('stop');
        }
    }, 100); // 1/10th of a second
}

// Function to handle button press for movement
function handleButtonPress(event) {
    if (!commandActive) {
        clearInterval(stopCommandInterval);
        
        // Get the closest button element to ensure correct command is retrieved
        var button = event.target.closest('.button');
        var command = button.getAttribute('data-command');
        
        if (command) {
            send_cmd(command);
            commandActive = true; // Set flag to indicate command is active
        }
    }
}


// Function to handle button release
function handleButtonRelease() {
    if (commandActive) {
        send_cmd('stop');
        commandActive = false; // Reset flag as command is no longer active
        startStopCommandInterval();
    }
}

// Attach event listeners to buttons
var buttons = document.querySelectorAll('.button');
buttons.forEach(function(button) {
    button.addEventListener('mousedown', handleButtonPress);
    button.addEventListener('mouseup', handleButtonRelease);
    button.addEventListener('touchstart', handleButtonPress);
    button.addEventListener('touchend', handleButtonRelease);
});

// Handle cases where the mouse is released away from the button
document.addEventListener('mouseup', handleButtonRelease);
document.addEventListener('touchend', handleButtonRelease);
</script>



</body>
</html>


)rawliteral";
/* ======================================== */

/* ________________________________________________________________________________ Index handler function to be called during GET or uri request */
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ stream handler function to be called during GET or uri request. */
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  /* ---------------------------------------- Loop to show streaming video from ESP32 Cam camera. */
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed (stream_handler)");
      res = ESP_FAIL;
    } else {
      if (fb->width > 400) {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
  }
  /* ---------------------------------------- */
  return res;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ cmd handler function to be called during GET or uri request. */
static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf;
  size_t buf_len;
  char variable[32] = {
    0,
  };

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int res = 0;

  Serial.print("Incoming command : ");
  Serial.println(variable);
  Serial.println();
  String getData = String(variable);
  String resultData = getValue(getData, ',', 0);

  /* ---------------------------------------- Controlling the LEDs on the ESP32 Cam board with PWM. */
  // Example :
  // Incoming command = S,10
  // S = Slider
  // 10 = slider value
  // I set the slider value range from 0 to 20.
  // Then the slider value is changed from 0 - 20 or vice versa to 0 - 255 or vice versa.
  if (resultData == "S") {
    resultData = getValue(getData, ',', 1);
    int pwm = map(resultData.toInt(), 0, 20, 0, 255);
    ledcSetup(ledChannel, PWM_freq, PWM_resolution);
    ledcAttachPin(LED_OnBoard, ledChannel);
    ledcWrite(ledChannel, pwm);
    Serial.print("PWM LED On Board : ");
    Serial.println(pwm);
  }

  if (getData == "forwards") {
    acceleratingForwards = true;
    acceleratingBackwards = false;
  } else if (getData == "backwards") {
    acceleratingBackwards = true;
    acceleratingForwards = false;
  } else if (getData == "left") {
    acceleratingLeft = true;
    acceleratingRight = false;
  } else if (getData == "right") {
    acceleratingRight = true;
    acceleratingLeft = false;
  }

  if (getData == "turret_left") {
    turretMovingLeft = true;
    turretMovingRight = false;
  } else if (getData == "turret_right") {
    turretMovingRight = true;
    turretMovingLeft = false;
  }

  else if (getData == "stop") {
    // Stop all acceleration
    acceleratingForwards = false;
    acceleratingBackwards = false;
    acceleratingLeft = false;
    acceleratingRight = false;
    turretMovingRight = false;
    turretMovingLeft = false;
  }
  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

void updateSpeed() {
  // Apply acceleration based on the direction
  if (acceleratingForwards) {
    current_speed_left_f += ACCELERATION;
    current_speed_right_f -= ACCELERATION;
  }
  if (acceleratingBackwards) {
    current_speed_left_f -= ACCELERATION;
    current_speed_right_f += ACCELERATION;
  }
  if (acceleratingLeft) {
    current_speed_left_f -= ACCELERATION;
    current_speed_right_f -= ACCELERATION;
  }
  if (acceleratingRight) {
    current_speed_left_f += ACCELERATION;
    current_speed_right_f += ACCELERATION;
  }

  // Apply drag (deceleration) when not accelerating
  if (!acceleratingForwards && !acceleratingBackwards && !acceleratingLeft && !acceleratingRight) {
    if (current_speed_left_f != STOP) {
      current_speed_left_f += (current_speed_left_f > STOP ? -DRAG : DRAG);
    }
    if (current_speed_right_f != STOP) {
      current_speed_right_f += (current_speed_right_f > STOP ? -DRAG : DRAG);
    }
  }

  // Clamp speeds within the min and max range
  current_speed_left_f = max(min(current_speed_left_f, MAX_SPEED_FORWARD), MAX_SPEED_BACKWARD);
  current_speed_right_f = max(min(current_speed_right_f, MAX_SPEED_FORWARD), MAX_SPEED_BACKWARD);

  // Convert to integer and update servo speeds
  servo_left.writeMicroseconds((int)current_speed_left_f);
  servo_right.writeMicroseconds((int)current_speed_right_f);
}

void updateTurretPosition() {
  // Apply movement speed
  if (turretMovingLeft) {
    turret_current_speed -= TURRET_ACCELERATION;
  } else if (turretMovingRight) {
    turret_current_speed += TURRET_ACCELERATION;
  } else {
    // Apply drag when the turret is not moving
    if (turret_current_speed != STOP) {
      turret_current_speed += (turret_current_speed > STOP ? -DRAG : DRAG);
    }
  }

  // Clamp the turret speed to prevent it from exceeding the max or min speeds
  turret_current_speed = max(min(turret_current_speed, turret_max_speed), turret_min_speed);

  // Convert the speed to an angle within the servo's range if necessary
  // This might involve mapping turret_current_speed to a valid servo microsecond range
  //int turret_servo_position = map(turret_current_speed, turret_min_speed, turret_max_speed, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);

  // Update the turret servo position
  servo_turret.writeMicroseconds(turret_current_speed);
}


void startCameraWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri = "/action",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&index_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(index_httpd, &index_uri);
    httpd_register_uri_handler(index_httpd, &cmd_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }

  Serial.println();
  Serial.println("Camera Server started successfully");
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(WiFi.localIP());
  Serial.println();
}

String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


void setup() {
  // put your setup code here, to run once:

  // Disable brownout detector.
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  /* ---------------------------------------- Init serial communication speed (baud rate). */
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();
  /* ---------------------------------------- */

  pinMode(LED_OnBoard, OUTPUT);

  /* ---------------------------------------- Setting Servos */
  Serial.println();
  Serial.println("------------");
  Serial.println("Start setting Servos...");


  //ESP32PWM::allocateTimer(0); // Allocate timer
  //ESP32PWM::allocateTimer(1); // Allocate timer
  ESP32PWM::allocateTimer(2); // Allocate timer
  ESP32PWM::allocateTimer(3); // Allocate timer

  //ESP32PWM::timerCount[2]=4;
  //ESP32PWM::timerCount[3]=4;

  servo_right.setPeriodHertz(50);   //--> // Standard 50hz servo
  servo_right.attach(servo_right_Pin, 1000, 2000);

  servo_left.setPeriodHertz(50);    //--> // Standard 50hz servo
  servo_left.attach(servo_left_Pin, 1000, 2000);

  servo_turret.setPeriodHertz(50);  //--> // Standard 50hz servo
  servo_turret.attach(servo_turret_Pin, 1000, 2000);



  //servo_Dummy_1.attach(15, 1000, 2000);  //these two dummys are something to do with avoiding timers used for the camera
  //servo_Dummy_2.attach(13, 1000, 2000);


  //servo_right.write(servo_right_Pos);
  //servo_left.write(servo_left_Pos);
  //servo_turret.writeMicroseconds(1500);

  Serial.println("Setting up the servos was successful.");
  Serial.println("------------");
  Serial.println();
  /* ---------------------------------------- */

  delay(2000);

  /* ---------------------------------------- Camera configuration. */
  Serial.println();
  Serial.println("------------");
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }

  // Get the camera sensor configuration
  sensor_t *s = esp_camera_sensor_get();

  // Set horizontal and vertical flip
  s->set_hmirror(s, 1);  // Set to 1 to enable horizontal mirror, 0 to disable
  s->set_vflip(s, 1);    // Set to 1 to enable vertical flip, 0 to disable


  Serial.println("Configure and initialize the camera successfully.");
  Serial.println("------------");
  Serial.println();
  /* ---------------------------------------- */

  /* ---------------------------------------- Connect to Wi-Fi. */
  WiFi.mode(WIFI_STA);
  Serial.println("------------");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  /* ::::::::::::::::: The process of connecting ESP32 CAM with WiFi Hotspot / WiFi Router. */
  /*
   * The process timeout of connecting ESP32 CAM with WiFi Hotspot / WiFi Router is 20 seconds.
   * If within 20 seconds the ESP32 CAM has not been successfully connected to WiFi, the ESP32 CAM will restart.
   * I made this condition because on my ESP32-CAM, there are times when it seems like it can't connect to WiFi, so it needs to be restarted to be able to connect to WiFi.
   */
  int connecting_process_timed_out = 20;  //--> 20 = 20 seconds.
  connecting_process_timed_out = connecting_process_timed_out * 2;
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    digitalWrite(LED_OnBoard, HIGH);
    delay(250);
    digitalWrite(LED_OnBoard, LOW);
    delay(250);
    if (connecting_process_timed_out > 0) connecting_process_timed_out--;
    if (connecting_process_timed_out == 0) {
      delay(1000);
      ESP.restart();
    }
  }
  digitalWrite(LED_OnBoard, LOW);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("------------");
  Serial.println("");
  /* ::::::::::::::::: */
  /* ---------------------------------------- */

  // Start camera web server
  startCameraWebServer();
}



void loop() {
  // put your main code here, to run repeatedly:
  updateSpeed();
  updateTurretPosition();
  delay(1);
}