#include <ARDUINO.h>
#include <motor.h>
#include <PID.h>

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

AsyncWebServer server(80);
const char *ssid = "bask_network";     // Your WiFi SSID
const char *password = "virtual09090"; // Your WiFi Password

// {ena,pin1,pin2}
int right_wheel[3] = {2, 15, 13};
int left_wheel[3] = {17, 4, 16};

// sensor pins
int ir_sensor[4] = {39, 36, 35, 34};

// setting PWM properties
const double freq = 30000;
const uint8_t left_motor_channel = 2;
const uint8_t right_motor_channel = 1;
const uint8_t resolution = 8;

// PID Constants
float Kp = 5;
float Ki = 1;
float Kd = 5;

float error = 0;
int flag = 0;
int run_SW_pin = 32;
int led_blue = 12;
int led_yellow = 27;

int left_motor_speed = 5;
int right_motor_speed = 5;
int initial_speed = 5;

Motor left_motor(left_wheel, left_motor_channel);
Motor right_motor(right_wheel, right_motor_channel);
PID pid(Kp, Ki, Kd);

int value = 0;
boolean u_turn = true;
int left_turn_speed[2] = {0, 0};
int right_turn_speed[2] = {0, 0};

void recvMsg(uint8_t *data, size_t len)
{
  WebSerial.println("Received Data...");
  String d = "";
  for (int i = 0; i < len; i++)
  {
    d += char(data[i]);
  }
  WebSerial.println(d);
  if (d == "P_up")
    pid.Kp++;

  if (d == "P_down")
    pid.Kp--;

  if (d == "D_up")
    pid.Kd++;

  if (d == "D_down")
    pid.Kd--;

  if (d == "I_up")
    pid.Ki++;

  if (d == "I_down")
    pid.Ki--;

  if (d == "speed_up")
  {
    left_motor_speed += 10;
    right_motor_speed += 10;
  }
  if (d == "u_turn")
  {
    WebSerial.print("turn: ");
    WebSerial.print(u_turn);
    u_turn = !u_turn;
  }

  if (d == "speed_down")
  {
    left_motor_speed -= 10;
    right_motor_speed -= 10;
  }

  if (d == "rtlu")
  {
    right_turn_speed[0] += 5;
  }

  if (d == "rtru")
  {
    right_turn_speed[1] += 5;
  }

  if (d == "rtld")
  {
    right_turn_speed[0] -= 5;
  }

  if (d == "rtrd")
  {
    right_turn_speed[1] -= 5;
  }

  if (d == "ltlu")
  {
    left_turn_speed[0] += 5;
  }

  if (d == "ltru")
  {
    left_turn_speed[1] += 5;
  }

  if (d == "ltld")
  {
    left_turn_speed[0] -= 5;
  }

  if (d == "ltrd")
  {
    left_turn_speed[1] -= 5;
  }

  WebSerial.print("P: ");
  WebSerial.print(pid.Kp);
  WebSerial.print(" I: ");
  WebSerial.print(pid.Ki);
  WebSerial.print(" D: ");
  WebSerial.print(pid.Kd);
  WebSerial.print("\t");
  WebSerial.print(left_motor_speed);
  WebSerial.print("\t");
  WebSerial.println(right_motor_speed);
  WebSerial.print(" turn: ");
  WebSerial.println(u_turn);
  WebSerial.print(" left turn speed: ");
  WebSerial.print(left_turn_speed[0]);
  WebSerial.print("\t");
  WebSerial.print(left_turn_speed[1]);
  WebSerial.print(" right turn speed: ");
  WebSerial.print(right_turn_speed[0]);
  WebSerial.print("\t");
  WebSerial.println(right_turn_speed[1]);
}

void setup()
{
  Serial.begin(115200);
  right_motor.setup();
  left_motor.setup();

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.printf("WiFi Failed!\n");
    return;
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  // WebSerial is accessible at "<IP Address>/webserial" in browser
  WebSerial.begin(&server);
  WebSerial.msgCallback(recvMsg);
  server.begin();

  pinMode(run_SW_pin, INPUT);
  pinMode(led_blue, OUTPUT);
  pinMode(led_yellow, OUTPUT);

  pinMode(14, OUTPUT);
  pinMode(26, OUTPUT);

  digitalWrite(14, LOW);
  digitalWrite(26, LOW);

  // set pins as output
  for (int i = 1; i < 3; i++)
  {
    pinMode(left_wheel[i], OUTPUT);
    pinMode(right_wheel[i], OUTPUT);
  }
  // set sensor pins as input
  for (int i = 0; i < 4; i++)
  {
    pinMode(ir_sensor[i], INPUT);
  }

  // initialize pins to LOW
  for (int i = 1; i < 3; i++)
  {
    digitalWrite(left_wheel[i], LOW);
    digitalWrite(right_wheel[i], LOW);
  }

  // configure PWM functionalitites
  ledcSetup(right_motor_channel, freq, resolution);
  ledcSetup(left_motor_channel, freq, resolution);

  left_motor.speed = initial_speed;
  right_motor.speed = initial_speed;
  left_motor.min_threshold = 145;
  right_motor.min_threshold = 145;
}

void loop()
{

  if (digitalRead(run_SW_pin) == HIGH)
  {
    run_robot();
    digitalWrite(led_yellow, HIGH);
    digitalWrite(led_blue, LOW);
  }
  else
  {
    // Serial.println("Un-armed");
    digitalWrite(led_blue, HIGH);
    digitalWrite(led_yellow, LOW);
  }
}

void run_robot()
{

  read_sensor_values();
  Serial.print(error);

  Serial.print(" sensor values: ");
  for (int i = 0; i < 3; i++)
  {
    Serial.print(digitalRead(ir_sensor[i]));
    Serial.print(", ");
  }
  Serial.print(digitalRead(ir_sensor[3]));

  if (error == 100)
  { // Turn Left 90* Make left turn untill it detects straight path
    Serial.print("\t");
    Serial.print("turning right");
    // do
    // {                                         // Turn left until you find the middle of the line
      left_motor.speed = left_turn_speed[0];  // Left Motor Speed
      right_motor.speed = left_turn_speed[1]; // Right Motor Speed
      right();
      read_sensor_values();

    // } while (error == 0);
  }
  else if (error == 101)
  { // Turn Right 90* Make right turn in case of it detects only right path (it will go into forward direction in case of staright and right "|--")
    // untill it detects straight path.
    Serial.print("\t");
    Serial.print("turning right");
    //  Turn your head to the left until the line is detected, then stop
    // do
    // {
      // Turn left until you find the middle of the line
      left_motor.speed = right_turn_speed[0];  // Left Motor Speed
      right_motor.speed = right_turn_speed[1]; // Right Motor Speed
      sharpRightTurn();
      read_sensor_values();
    // } while (error == 0);
  }
  else if (error == 102)
  { // Turn your head to the left  Make left turn untill it detects straight path
    Serial.print("\t");
    Serial.print("Sharp Left Turn");
    // do
    // {                                         // Turn left until you find the middle of the line
      left_motor.speed = left_turn_speed[0];  // Left Motor Speed
      right_motor.speed = left_turn_speed[1]; // Right Motor Speed
      sharpLeftTurn();
      read_sensor_values();
    // } while (error == 0 && u_turn);
  }
  else if (error == 103)
  { // Make left turn untill it detects straight path or stop if dead end reached.
    if (flag == 0)
    {

      left_motor.speed = left_motor_speed;   // Left Motor Speed
      right_motor.speed = right_motor_speed; // Right Motor Speed
      forward();
      delay(300);
      stop_bot();
      read_sensor_values();

      if (error == 103)
      { /**** Dead End Reached, Stop! ****/
        stop_bot();
        delay(200);
        flag = 1;
      }
      else
      { /**** Move Left ****/
        // do
        // { // Turn left until you find the middle of the line
          // Serial.print("\t");
          // Serial.print("Left Here");
          left_motor.speed = left_turn_speed[0];  // Left Motor Speed
          right_motor.speed = left_turn_speed[1]; // Right Motor Speed
          sharpLeftTurn();
          read_sensor_values();
        // } while (error == 0);
      }
    }
  }
  else
  {
    float PID_value = pid.calculate(error); // Calculate PID value
    Serial.print("\t PID value: ");
    Serial.print(PID_value);

    left_motor.PID_value = -PID_value;
    right_motor.PID_value = PID_value;

    forward();
  }

  Serial.println(";");
}

void read_sensor_values()
{

  const int right_sensor_side = digitalRead(ir_sensor[0]);
  const int right_sensor_center = digitalRead(ir_sensor[1]);
  const int left_sensor_center = digitalRead(ir_sensor[2]);
  const int left_sensor_side = digitalRead(ir_sensor[3]);

  if ((right_sensor_side == 1) && (right_sensor_center == 0) && (left_sensor_center == 0) && (left_sensor_side == 0)) // much right
    error = -3;
  else if ((right_sensor_side == 1) && (right_sensor_center == 1) && (left_sensor_center == 0) && (left_sensor_side == 0))
    error = -2;
  else if ((right_sensor_side == 0) && (right_sensor_center == 1) && (left_sensor_center == 0) && (left_sensor_side == 0))
    error = -1;
  else if ((right_sensor_side == 0) && (right_sensor_center == 1) && (left_sensor_center == 1) && (left_sensor_side == 0)) // no deviation
    error = 0;
  else if ((right_sensor_side == 0) && (right_sensor_center == 0) && (left_sensor_center == 1) && (left_sensor_side == 0))
    error = 1;
  else if ((right_sensor_side == 0) && (right_sensor_center == 0) && (left_sensor_center == 1) && (left_sensor_side == 1))
    error = 2;
  else if ((right_sensor_side == 0) && (right_sensor_center == 0) && (left_sensor_center == 0) && (left_sensor_side == 1)) // much left
    error = 3;
  else if ((right_sensor_side == 1) && (right_sensor_center == 1) && (left_sensor_center == 1) && (left_sensor_side == 0)) // Turn robot right side
    error = 100;
  else if ((right_sensor_side == 0) && (right_sensor_center == 1) && (left_sensor_center == 1) && (left_sensor_side == 1)) // Turn robot left side
    error = 101;
  else if ((right_sensor_side == 0) && (right_sensor_center == 0) && (left_sensor_center == 0) && (left_sensor_side == 0)) // Make U turn
    error = 102;
  else if ((right_sensor_side == 1) && (right_sensor_center == 1) && (left_sensor_center == 1) && (left_sensor_side == 1)) // Turn left side or stop
    error = 103;
}

void forward()
{
  right_motor.run();
  left_motor.run();
}
void reverse()
{
  right_motor.run(true);
  left_motor.run(true);
}
void right()
{
  right_motor.stop();
  left_motor.run();
}
void left()
{
  right_motor.run();
  left_motor.stop();
}
void sharpLeftTurn()
{
  right_motor.run();
  left_motor.run(true);
}
void sharpRightTurn()
{
  right_motor.run(true);
  left_motor.run();
}
void stop_bot()
{
  right_motor.stop();
  left_motor.stop();
}
