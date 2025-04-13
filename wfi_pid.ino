#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <WiFi.h>
#include <WebServer.h>

// MPU6050 and PID setup
MPU6050 mpu;

// Motor PWM pins
#define MOTOR1_PIN 13  // Front Left
#define MOTOR2_PIN 27  // Front Right
#define MOTOR3_PIN 14  // Rear Left
#define MOTOR4_PIN 12  // Rear Right

// Wi-Fi credentials
const char* ssid = "kd";
const char* password = "12345678";

// Web server on port 80
WebServer server(80);

// Global variables
double setpointX = 0, setpointY = 0;
double inputX, inputY;
double outputX, outputY;

// Base speed and PID values (modifiable via web)
int baseSpeed = 0;
double Kp = 3.5, Ki = 0.0, Kd = 0.0;

// PID objects
PID pidX(&inputX, &outputX, &setpointX, Kp, Ki, Kd, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, Kp, Ki, Kd, DIRECT);

// Kalman filter class
class Kalman {
public:
  float angle, bias, rate;
  float P[2][2];
  Kalman() {
    angle = 0;
    bias = 0;
    P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
  }

  float getAngle(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + 0.001);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += 0.0005;

    float S = P[0][0] + 0.1;
    float K[2] = { P[0][0] / S, P[1][0] / S };

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    P[0][0] -= K[0] * P[0][0];
    P[0][1] -= K[0] * P[0][1];
    P[1][0] -= K[1] * P[0][0];
    P[1][1] -= K[1] * P[0][1];

    return angle;
  }
};

Kalman kalmanX, kalmanY;

int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

void calibrateMPU6050() {
  int num_samples = 500;
  long ax_sum = 0, ay_sum = 0, az_sum = 0, gx_sum = 0, gy_sum = 0, gz_sum = 0;

  Serial.println("Calibrating MPU6050...");
  for (int i = 0; i < num_samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += ax; ay_sum += ay; az_sum += az;
    gx_sum += gx; gy_sum += gy; gz_sum += gz;
    delay(5);
  }

  ax_offset = ax_sum / num_samples;
  ay_offset = ay_sum / num_samples;
  az_offset = az_sum / num_samples - 16384;
  gx_offset = gx_sum / num_samples;
  gy_offset = gy_sum / num_samples;
  gz_offset = gz_sum / num_samples;

  Serial.println("Calibration done.");
}

// HTTP handler
void handleUpdate() {
  if (server.hasArg("base")) baseSpeed = server.arg("base").toInt();
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();

  pidX.SetTunings(Kp, Ki, Kd);
  pidY.SetTunings(Kp, Ki, Kd);

  server.send(200, "text/plain", "Updated values:\nbase=" + String(baseSpeed) +
              "\nKp=" + String(Kp) + "\nKi=" + String(Ki) + "\nKd=" + String(Kd));
}

// Timing
unsigned long timer;
float dt;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  // Motor pin setup
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);

  // Wi-Fi setup
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.println("IP Address: " + WiFi.localIP().toString());

  // Start WebServer
  server.on("/update", handleUpdate);
  server.begin();

  // MPU setup
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 connected.");
  calibrateMPU6050();

  // PID setup
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-100, 100);
  pidY.SetOutputLimits(-100, 100);

  timer = micros();
}

void loop() {
  server.handleClient();

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  ax -= ax_offset; ay -= ay_offset; az -= az_offset;
  gx -= gx_offset; gy -= gy_offset; gz -= gz_offset;

  float accel_angle_x = atan2(ay, az) * 180 / PI;
  float accel_angle_y = atan2(ax, az) * 180 / PI;

  float gyro_rate_x = gx / 131.0;
  float gyro_rate_y = gy / 131.0;

  dt = (micros() - timer) / 1000000.0;
  timer = micros();

  float kalman_angle_x = kalmanX.getAngle(accel_angle_x, gyro_rate_x, dt);
  float kalman_angle_y = kalmanY.getAngle(accel_angle_y, gyro_rate_y, dt);

  inputX = -kalman_angle_x * 0;  // not using roll right now
  inputY = kalman_angle_y;

  pidX.Compute();
  pidY.Compute();

  int motor1Speed = constrain(baseSpeed - outputX + outputY, 0, 70);
  int motor2Speed = constrain(baseSpeed + outputX + outputY, 0, 70);
  int motor3Speed = constrain(baseSpeed - outputX - outputY, 0, 70);
  int motor4Speed = constrain(baseSpeed + outputX - outputY, 0, 70);

  analogWrite(MOTOR1_PIN, motor1Speed);
  analogWrite(MOTOR2_PIN, motor2Speed);
  analogWrite(MOTOR3_PIN, motor3Speed);
  analogWrite(MOTOR4_PIN, motor4Speed);

  Serial.print("Y: "); Serial.print(kalman_angle_y);
  Serial.print(" OutputY: "); Serial.print(outputY);
  Serial.print(" Base: "); Serial.println(baseSpeed);
}
