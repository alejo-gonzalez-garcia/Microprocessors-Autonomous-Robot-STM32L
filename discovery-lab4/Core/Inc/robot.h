#ifndef __ROBOT_H
#define __ROBOT_H

#include "stm32l1xx_hal.h"

#define MAX_SPEED 100
#define TURN_SPEED 70
#define MIN_SPEED 30
#define MAX_VALUE_ADC 4095
#define SPEED_SOUND_CM_MS 0.034
#define LONG_DISTANCE 20
#define SHORT_DISTANCE 10

#define TIMER_2_PSC 32
#define TIMER_2_CH_2_CNT 10

#define TIMER_3_PSC 3200
#define TIMER_3_CH_1_CNT 4500
#define TIMER_3_CH_2_CNT 50
#define TIMER_3_CH_3_CNT 2250

#define TIMER_4_PSC 640

#define DATA_RECEIVED_LENGHT 1
#define DATA_SEND_LENGTH 50


extern struct Robot g_robot;

enum StatusGPIOPin { GPIO_PIN_UP, GPIO_PIN_DOWN};
enum StatusMotor { MOTOR_STOPPED, MOTOR_FORWARD, MOTOR_BACKWARD };
enum StatusUltrasound { ULTRASOUND_STOPPED, ULTRASOUND_TRIGGER_START, ULTRASOUND_TRIGGER_ON, ULTRASOUND_TRIGGER_SENT, ULTRASOUND_MEASURING };
enum StatusDistance { DISTANCE_NONE, DISTANCE_10, DISTANCE_20 };
enum StatusBuzzer { BUZZER_OFF, BUZZER_ON, BUZZER_BEEPING };
enum StatusDelay {DELAY_START, DELAY_WAITING, DELAY_OFF };

enum StatusObstacle { OBSTACLE_NONE, OBSTACLE_IN_FRONT, OBSTACLE_RIGHT, OBSTACLE_RIGHT_MEASURE, OBSTACLE_RIGHT_BACK,
    OBSTACLE_LEFT, OBSTACLE_LEFT_MEASURE, OBSTACLE_LEFT_BACK, OBSTACLE_FINAL };
enum StatusMode { MODE_DEFAULT, MODE_AUTOMATIC, MODE_MANUAL };
enum StatusRobot { ROBOT_DEFAULT, ROBOT_STOPPED, ROBOT_FORWARD, ROBOT_BACKWARD, ROBOT_RIGHT, ROBOT_LEFT, ROBOT_BACKWARD_RIGHT, ROBOT_BACKWARD_LEFT};

struct GPIOPin {
  GPIO_TypeDef *gpio;
  unsigned char pin;
};

struct Motor {
  enum StatusMotor status;
  struct GPIOPin pin_speed;
  struct GPIOPin pin_direction;
  unsigned char channel;
};

struct Ultrasound {
  enum StatusUltrasound status;
  enum StatusDistance status_distance;
  struct GPIOPin *trigger;
  struct GPIOPin *echo;
  uint16_t time_init;
  int distance;
};

struct Buzzer {
  enum StatusBuzzer status;
  struct GPIOPin *gpio_pin;
};

struct SpeedSelector {
  unsigned short max_speed;
  struct GPIOPin *gpio_pin;
  ADC_TypeDef *adc;
};

struct Bluetooth {
  UART_HandleTypeDef *huart;
  char *data_send, *data_received;
};

struct Robot {
  enum StatusRobot status_robot;
  enum StatusObstacle status_obstacle;
  enum StatusMode status_mode;
  enum StatusDelay delay;
  unsigned char speed;
  struct Motor *motor_right;
  struct Motor *motor_left;
  struct Buzzer *buzzer;
  struct Ultrasound *ultrasound;
  struct SpeedSelector speed_selector;
  struct Bluetooth bluetooth;
};

void createRobot(UART_HandleTypeDef *huart);

void updateStatusGPIOPin(struct GPIOPin *gpio_pin, enum StatusGPIOPin status);
void toggleGPIOPin(struct GPIOPin *gpio_pin);

void updateStatusBuzzer(enum StatusBuzzer status);
void updateBuzzer();
void updateMaxSpeed();

void updateStatusMode(enum StatusMode status);

void updateStatusRobot(enum StatusRobot status);
void updateSpeedRobot(unsigned char speed);
void updateRobot();

void receiveData();

void sendDistanceData();
void sendData();



#endif /* __ROBOT_H */
