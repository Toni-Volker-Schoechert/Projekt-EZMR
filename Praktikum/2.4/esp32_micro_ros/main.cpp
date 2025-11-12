#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h> 
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#define MOTOR_TX_PIN 17
#define START_BYTE 0xAA

#define BUMPER_LEFT_PIN 14
#define BUMPER_RIGHT_PIN 12

#define BUZZER_PIN 4
#define BUZZER_CH 0

HardwareSerial Serial2_ = HardwareSerial(2);

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Subscribers
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t buzzer_subscriber;

// Publishers
rcl_publisher_t debug_publisher;
rcl_publisher_t bumper_left_pub;
rcl_publisher_t bumper_right_pub;

// Timer (zum Polling der Bumper)
rcl_timer_t bumper_timer;     

geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int32 buzzer_msg;
std_msgs__msg__String debug_msg;
std_msgs__msg__Bool bumper_left_msg;
std_msgs__msg__Bool bumper_right_msg;

// Bumper Debounce-Variablen
volatile bool last_left_state = false;
volatile bool last_right_state = false;
volatile bool stable_left_state = false;
volatile bool stable_right_state = false;
unsigned long last_left_change = 0;
unsigned long last_right_change = 0;
const unsigned long DEBOUNCE_MS = 50; // 50 ms Debounce

//Buzzer var
unsigned long buzzer_end = 0;
bool buzzer_active = false;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1) delay(100);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// UART-Motorbefehl senden
void sendMotorCommand(uint8_t id, uint8_t mode, uint8_t dir, uint16_t param) {
  uint8_t packet[6];
  packet[0] = START_BYTE;
  packet[1] = id;
  packet[2] = mode;
  packet[3] = dir;
  packet[4] = param & 0xFF;
  packet[5] = (param >> 8) & 0xFF;
  Serial2_.write(packet, 6);
}

// Callback für /cmd_vel
void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * received_msg = (const geometry_msgs__msg__Twist *)msgin;

    float linear = received_msg->linear.x;
    float angular = received_msg->angular.z;

    // einfache Differentialsteuerung
    int16_t left_motor = linear - angular;
    int16_t right_motor = linear + angular;

    // Begrenzung
    left_motor = constrain(left_motor, -100, 100);
    right_motor = constrain(right_motor, -100, 100);

    uint8_t left_dir = (left_motor >= 0) ? 1 : 0;//Motoren müssen engegengesetz dir haben um gleich zu bewegen(gradeaus)
    uint8_t right_dir = (right_motor >= 0) ? 0 : 1;
    uint16_t left_speed = abs(left_motor);
    uint16_t right_speed = abs(right_motor);

    // Motorbefehle senden
    sendMotorCommand(1, 0, left_dir, left_speed);
    sendMotorCommand(2, 0, right_dir, right_speed);

    // Debug als String aufbereiten
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "cmd_vel -> L:%d(%d) R:%d(%d)", 
           left_speed, left_dir, right_speed, right_dir);
    debug_msg.data.data = buffer;
    debug_msg.data.size = strlen(buffer);
    debug_msg.data.capacity = strlen(buffer) + 1;

    // Publish Debug-Nachricht über Micro-ROS
    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
}

// Buzzer Callback
void buzzer_callback(const void * msgin) {
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*) msgin;

    int freq = msg->data;     // Tonhöhe von der Nachricht
    int vol  = 1000;           // feste Lautstärke
    int dur  = 200;           // feste Dauer in ms

    // Debug-Ausgabe über ROS
    char buf[100];
    snprintf(buf, sizeof(buf), "Buzzer Callback: freq=%d, vol=%d, dur=%d", freq, vol, dur);
    debug_msg.data.data = buf;
    debug_msg.data.size = strlen(buf);
    debug_msg.data.capacity = strlen(buf) + 1;
    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));

    ledcWriteTone(BUZZER_CH, freq);
    ledcWrite(BUZZER_CH, vol);

    buzzer_end = millis() + dur;
    buzzer_active = true;
}

// BUMPER 
// Timer-callback prüft Bumper regelmäßig
void bumper_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  // Rohzustand (INPUT_PULLUP => HIGH = offen, LOW = gedrückt)
  bool raw_left = digitalRead(BUMPER_LEFT_PIN) == LOW;  // true = gedrückt
  bool raw_right = digitalRead(BUMPER_RIGHT_PIN) == LOW;

  unsigned long now = millis();

  // LEFT
  if (raw_left != last_left_state) {
    last_left_change = now;
    last_left_state = raw_left;
  } else {
    // Wenn seit Änderung > debounce und stabil != raw -> übernehmen & publizieren
    if ((now - last_left_change) > DEBOUNCE_MS && stable_left_state != raw_left) {
      stable_left_state = raw_left;
      // publish
      bumper_left_msg.data = stable_left_state;
      RCSOFTCHECK(rcl_publish(&bumper_left_pub, &bumper_left_msg, NULL));
      // debug
      char buf[80];
      snprintf(buf, sizeof(buf), "Bumper LEFT: %s", stable_left_state ? "PRESSED" : "RELEASED");
      debug_msg.data.data = buf;
      debug_msg.data.size = strlen(buf);
      debug_msg.data.capacity = strlen(buf) + 1;
      RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
    }
  }

  // RIGHT
  if (raw_right != last_right_state) {
    last_right_change = now;
    last_right_state = raw_right;
  } else {
    if ((now - last_right_change) > DEBOUNCE_MS && stable_right_state != raw_right) {
      stable_right_state = raw_right;
      bumper_right_msg.data = stable_right_state;
      RCSOFTCHECK(rcl_publish(&bumper_right_pub, &bumper_right_msg, NULL));
      char buf[80];
      snprintf(buf, sizeof(buf), "Bumper RIGHT: %s", stable_right_state ? "PRESSED" : "RELEASED");
      debug_msg.data.data = buf;
      debug_msg.data.size = strlen(buf);
      debug_msg.data.capacity = strlen(buf) + 1;
      RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
    }
  }
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    // Micro-ROS Serial Transport (USB)
    set_microros_serial_transports(Serial);
    delay(2000);

    // Motor-UART initialisieren
    Serial2_.begin(9600, SERIAL_8N1, -1, MOTOR_TX_PIN);

    // Bumper pins als INPUT_PULLUP
    pinMode(BUMPER_LEFT_PIN, INPUT_PULLUP);
    pinMode(BUMPER_RIGHT_PIN, INPUT_PULLUP);

    // Buzzer setup
    ledcSetup(BUZZER_CH, 2000, 10); 
    ledcAttachPin(BUZZER_PIN, BUZZER_CH);

    // initial states
    last_left_state = (digitalRead(BUMPER_LEFT_PIN) == LOW);
    last_right_state = (digitalRead(BUMPER_RIGHT_PIN) == LOW);
    stable_left_state = last_left_state;
    stable_right_state = last_right_state;

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_robot_node", "", &support));

    // Subscriber /cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    ));

    // Subscriber /buzzer
    RCCHECK(rclc_subscription_init_default(
        &buzzer_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/buzzer"
    ));

    // Publisher /debug
    RCCHECK(rclc_publisher_init_default(
        &debug_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/debug"
    ));

    // Publisher /bumper/left (Bool)
    RCCHECK(rclc_publisher_init_default(
        &bumper_left_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/bumper/left"
    ));

    // Publisher /bumper/right (Bool)
    RCCHECK(rclc_publisher_init_default(
        &bumper_right_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/bumper/right"
    ));

    // Timer für Bumper-Polling
    const unsigned int bumper_period_ms = 50;
    RCCHECK(rclc_timer_init_default(&bumper_timer, &support, RCL_MS_TO_NS(bumper_period_ms), bumper_timer_callback));


    // Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &buzzer_subscriber, &buzzer_msg, &buzzer_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &bumper_timer));
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
    
    if (buzzer_active && millis() >= buzzer_end) {
        ledcWrite(BUZZER_CH, 0);  // Buzzer aus
        buzzer_active = false;
    }

    delay(5);
}