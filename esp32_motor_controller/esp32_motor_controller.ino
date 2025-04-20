#include <Arduino.h>

// === PIN DEFINITIONS ===
// Motor Driver 1 Pins
#define PIN_MOTOR1_AIN1 26
#define PIN_MOTOR1_AIN2 27
#define PIN_MOTOR1_PWM  25
// Motor Driver 2 Pins
#define PIN_MOTOR2_BIN1 32
#define PIN_MOTOR2_BIN2 14
#define PIN_MOTOR2_PWM  33
// Motor Driver 3 Pins
#define PIN_MOTOR3_CIN1 17
#define PIN_MOTOR3_CIN2 16
#define PIN_MOTOR3_PWM  23
// Common Standby Pin
#define PIN_MOTOR_STBY 13

// === PWM CONFIGURATION ===
#define PWM_FREQ         1000
#define PWM_RESOLUTION   8     // PWM resolution 8 bit
#define PWM_MAX_DUTY     ((1 << PWM_RESOLUTION) - 1) // Max duty cycle (255)

// LEDC PWM channels
#define PWM_CHANNEL_M1   0
#define PWM_CHANNEL_M2   1
#define PWM_CHANNEL_M3   2

// === SERIAL COMMUNICATION ===
#define SERIAL_BAUD_RATE 115200
String serialCommandBuffer = "";

// Function Prototype
void processCommand(String command);
void setMotorState(int motor_number, char direction_char, int speed_percent);

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  serialCommandBuffer.reserve(50);
  delay(1000);
  Serial.println("\n--- ESP32 ROS2 Motor Controller Ready ---");
  Serial.printf("Listening on Serial at %d baud.\n", SERIAL_BAUD_RATE);
  Serial.println("Waiting for commands like: M<num>:<dir>:<speed>%");
  Serial.println("  <num>: 1-3");
  Serial.println("  <dir>: F, R, B, S");
  Serial.println("  <speed>: 0-100");
  Serial.println("----------------------------------------");


  // --- Initialize GPIO Pins ---
  Serial.println("Initializing GPIO pins...");
  // Standby Pin
  pinMode(PIN_MOTOR_STBY, OUTPUT);
  digitalWrite(PIN_MOTOR_STBY, HIGH);

  // Motor Pins (Initial LOW state)
  pinMode(PIN_MOTOR1_AIN1, OUTPUT); pinMode(PIN_MOTOR1_AIN2, OUTPUT);
  digitalWrite(PIN_MOTOR1_AIN1, LOW); digitalWrite(PIN_MOTOR1_AIN2, LOW);
  pinMode(PIN_MOTOR2_BIN1, OUTPUT); pinMode(PIN_MOTOR2_BIN2, OUTPUT);
  digitalWrite(PIN_MOTOR2_BIN1, LOW); digitalWrite(PIN_MOTOR2_BIN2, LOW);
  pinMode(PIN_MOTOR3_CIN1, OUTPUT); pinMode(PIN_MOTOR3_CIN2, OUTPUT);
  digitalWrite(PIN_MOTOR3_CIN1, LOW); digitalWrite(PIN_MOTOR3_CIN2, LOW);

  // --- Initialize PWM (LEDC) ---
  Serial.println("Initializing PWM channels...");
  // Motor 1 PWM
  ledcSetup(PWM_CHANNEL_M1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_MOTOR1_PWM, PWM_CHANNEL_M1); ledcWrite(PWM_CHANNEL_M1, 0);
  // Motor 2 PWM
  ledcSetup(PWM_CHANNEL_M2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_MOTOR2_PWM, PWM_CHANNEL_M2); ledcWrite(PWM_CHANNEL_M2, 0);
  // Motor 3 PWM
  ledcSetup(PWM_CHANNEL_M3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_MOTOR3_PWM, PWM_CHANNEL_M3); ledcWrite(PWM_CHANNEL_M3, 0);

  Serial.println("Initialization Complete. Waiting for commands...");
}

void loop() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();

    if (incomingChar == '%') {
      // Command received, process it
      if (serialCommandBuffer.length() > 0) {
        processCommand(serialCommandBuffer);
      }
      serialCommandBuffer = "";
    } else {
      if (incomingChar != '\r' && incomingChar != '\n') {
         serialCommandBuffer += incomingChar;
         if (serialCommandBuffer.length() >= 49) {
            Serial.println("ERROR: Command buffer overflow! Clearing.");
            serialCommandBuffer = "";
         }
      }
    }
  }
   delay(5);
}

void processCommand(String command) {
  Serial.print("Received raw command: ");
  Serial.println(command);

  if (!command.startsWith("M") || command.indexOf(':') == -1 || command.lastIndexOf(':') == command.indexOf(':')) {
    Serial.println("ERROR: Invalid command format.");
    return;
  }

  int motor_num = 0;
  char dir_char = ' ';
  int speed_perc = -1;
  int parsed_items = sscanf(command.c_str(), "M%d:%c:%d", &motor_num, &dir_char, &speed_perc);

  if (parsed_items != 3) {
      Serial.printf("ERROR: Could not parse command. Items parsed: %d\n", parsed_items);
      return;
  }

  //validate
  if (motor_num < 1 || motor_num > 3) {
    Serial.printf("ERROR: Invalid motor number (%d).\n", motor_num);
    return;
  }
   if (dir_char != 'F' && dir_char != 'R' && dir_char != 'B' && dir_char != 'S') {
     Serial.printf("ERROR: Invalid direction character ('%c').\n", dir_char);
     return;
   }
   if (speed_perc < 0 || speed_perc > 100) {
     Serial.printf("ERROR: Invalid speed percentage (%d).\n", speed_perc);
     return;
   }

   //exec
   Serial.printf("Executing: Motor=%d, Dir=%c, Speed=%d%%\n", motor_num, dir_char, speed_perc);
   setMotorState(motor_num, dir_char, speed_perc);
}


/**
 * command
 *
 * motor_number 1, 2, or 3
 * direction_char 'F', 'R', 'B', or 'S'
 * speed_percent Speed as a % (0-100).
 */
void setMotorState(int motor_number, char direction_char, int speed_percent) {
  int pin_in1, pin_in2, pwm_channel;

  switch (motor_number) {
    case 1:
      pin_in1 = PIN_MOTOR1_AIN1; pin_in2 = PIN_MOTOR1_AIN2; pwm_channel = PWM_CHANNEL_M1; break;
    case 2:
      pin_in1 = PIN_MOTOR2_BIN1; pin_in2 = PIN_MOTOR2_BIN2; pwm_channel = PWM_CHANNEL_M2; break;
    case 3:
      pin_in1 = PIN_MOTOR3_CIN1; pin_in2 = PIN_MOTOR3_CIN2; pwm_channel = PWM_CHANNEL_M3; break;
    default:
      Serial.printf("INTERNAL ERROR: Invalid motor number %d in setMotorState\n", motor_number);
      return;
  }

  //clamp
  if (speed_percent < 0) speed_percent = 0;
  if (speed_percent > 100) speed_percent = 100;

  //map
  int duty = map(speed_percent, 0, 100, 0, PWM_MAX_DUTY);

  switch (direction_char) {
    case 'F': //forward
      digitalWrite(pin_in1, HIGH);
      digitalWrite(pin_in2, LOW);
      ledcWrite(pwm_channel, duty);
      break;
    case 'R': //reverse
      digitalWrite(pin_in1, LOW);
      digitalWrite(pin_in2, HIGH);
      ledcWrite(pwm_channel, duty);
      break;
    case 'B': //brake
      digitalWrite(pin_in1, LOW);
      digitalWrite(pin_in2, LOW);
      ledcWrite(pwm_channel, 0);
      break;
    case 'S': //stop
      digitalWrite(pin_in1, LOW);
      digitalWrite(pin_in2, LOW);
      ledcWrite(pwm_channel, 0);
      break;
    default:
      // Should not happen
      Serial.printf("INTERNAL ERROR: Invalid direction '%c'\n", direction_char);
      digitalWrite(pin_in1, LOW);
      digitalWrite(pin_in2, LOW);
      ledcWrite(pwm_channel, 0);
      break;
  }
}