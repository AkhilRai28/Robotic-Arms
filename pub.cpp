#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#define MOTOR1_PWM_PIN 13
#define MOTOR1_DIR_PIN 12
#define MOTOR2_PWM_PIN 14
#define MOTOR2_DIR_PIN 26
#define MOTOR3_PWM_PIN 25
#define MOTOR3_DIR_PIN 33
#define MOTOR4_PWM_PIN 5
#define MOTOR4_DIR_PIN 18
#define MOTOR5_PWM_PIN 19
#define MOTOR5_DIR_PIN 22

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop()
{
    while (1)
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

void setupMotors()
{
    pinMode(MOTOR1_PWM_PIN, OUTPUT);
    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_PWM_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    pinMode(MOTOR3_PWM_PIN, OUTPUT);
    pinMode(MOTOR3_DIR_PIN, OUTPUT);
    pinMode(MOTOR4_PWM_PIN, OUTPUT);
    pinMode(MOTOR4_DIR_PIN, OUTPUT);
    pinMode(MOTOR5_PWM_PIN, OUTPUT);
    pinMode(MOTOR5_DIR_PIN, OUTPUT);

    ledcSetup(0, 5000, 8); // Channel 0, 5 kHz PWM, 8-bit resolution
    ledcSetup(1, 5000, 8);
    ledcSetup(2, 5000, 8);
    ledcSetup(3, 5000, 8);
    ledcSetup(4, 5000, 8);

    ledcAttachPin(MOTOR1_PWM_PIN, 0);
    ledcAttachPin(MOTOR2_PWM_PIN, 1);
    ledcAttachPin(MOTOR3_PWM_PIN, 2);
    ledcAttachPin(MOTOR4_PWM_PIN, 3);
    ledcAttachPin(MOTOR5_PWM_PIN, 4);
}

void controlMotors(const geometry_msgs__msg__Twist *cmd)
{
    int speed = 128; 
    ledcWrite(0, cmd->linear.x * speed);
    ledcWrite(1, cmd->linear.y * speed);
    ledcWrite(2, cmd->linear.z * speed);
    ledcWrite(3, cmd->angular.x * speed);
    ledcWrite(4, cmd->angular.y * speed);
}

void subscription_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    controlMotors(msg);
}

void setup()
{
    Serial.begin(115200);
    set_microros_transports();
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(2000);

    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
    RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "manipulator_cmd"));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    setupMotors();
}

void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
