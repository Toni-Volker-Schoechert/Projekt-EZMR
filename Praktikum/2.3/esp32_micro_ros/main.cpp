#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h> // Für Debug-Topic
#include <sstream>

#define MOTOR_TX_PIN 17
#define START_BYTE 0xAA

HardwareSerial Serial2_ = HardwareSerial(2);

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t debug_publisher;

geometry_msgs__msg__Twist msg;
std_msgs__msg__String debug_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() { while(1) { delay(100); } }

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
    delay(5);
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

void setup() {
    Serial.begin(115200);
    delay(2000);

    // Micro-ROS Serial Transport (USB)
    set_microros_serial_transports(Serial);
    delay(2000);

    // Motor-UART initialisieren
    Serial2_.begin(9600, SERIAL_8N1, -1, MOTOR_TX_PIN);
    delay(100);


    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_cmd_vel_node", "", &support));

    // Subscriber /cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    ));

    // Publisher /debug
    RCCHECK(rclc_publisher_init_default(
        &debug_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/debug"
    ));

    // Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
    delay(50);
}
