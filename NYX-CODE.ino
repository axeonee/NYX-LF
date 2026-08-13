// ================= LIBRARIES =========================
#include <QTRSensors.h>
#include <NewPing.h>
#include <Servo.h>
#include "WiFiS3.h"
#include "web_site.h"
// ================= PID OBJECT ========================
#define MAX_SPEED_OBJ 170
#define MIN_SPEED_OBJ 0
#define OBJ_SETPOINT 700
#define SET_SPEED_OBJ 70

float Kp_obj = 0.85;
float Ki_obj = 0.000008;
float Kd_obj = 6;

int P_obj = 0, I_obj = 0, D_obj = 0;

int err_obj     = 0;
int prevErr_obj = 0;
int PID_obj     = 0;

int distance;
// ================= ULTRASONIC ========================
// Pini senzor distanță
#define TRIGGER_PIN 6
#define ECHO_PIN 7
int vcc  = 10;
// ================= MOTORS ============================
// Pin servo
int srv_mtr = 2;

// Direcție + frână
int dir_L   = 13;
int break_L = 8;
int dir_R   = 12;
int break_R = 9;

// Array pentru inițializare rapidă pini
int motors[] = { dir_L, dir_R, break_L, break_R };

// Pini viteză PWM
int speed_L = 11;
int speed_R = 3;
// ================= LINE SENSORS ======================
// Ordinea senzorilor este importantă (dreapta -> stânga)
#define sensorRR A5
#define sensorR  A4
#define sensorRM A3
#define sensorLM A2
#define sensorL  4
#define sensorLL 5

Servo ServoMtr;
NewPing sonar(TRIGGER_PIN, ECHO_PIN);
QTRSensors qtr;

const int sensorCount = 6;
uint16_t sensorValues[sensorCount];
// ================= PID LINE ===============================
#define MAX_SPEED 180
#define MIN_SPEED 0
#define SET_SPEED 90
#define SETPOINT 2500

float Kp = 0.12; // 1.00 Distanta ultimului senzor de la derivare este de 1.5cm // 0.85 Distanta e de 0.3cm asa ca am pus // 0.815 //ceea mai buna varianta 0.61
float Ki = 0.0001; // 0.0004 : 0.00006
float Kd = 1.3; // 0.275 Distanta ultimului senzor 0.1cm // 0.282 // 7.5 ceea mai bune valoare // 7.45 : 1.6

int P = 0, I = 0, D = 0;
int previousError = 0;
int PID_value     = 0;
int error         = 0;
// ================= STATES ============================
enum { followLine, avoidObstacle };
unsigned char roboState = followLine;
// ================= WIFI ==============================
char ssid[] = "NYX-Team-Wifi";
char pass[] = "nyx20025";

WiFiServer server(80);

// ================= SETUP =============================
void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    ServoMtr.attach(srv_mtr);
    ServoMtr.write(90);
    // ---------- QTR CALIBRATION ----------
    qtr.setTypeRC();
    qtr.setSensorPins(
        (const uint8_t[]){ sensorRR, sensorR, sensorRM, sensorLM, sensorL, sensorLL },
        sensorCount
    );
    // Calibrare senzori linie
    for (int i = 0; i < 250; i++) {
        qtr.calibrate();
        delay(20);
    }
    // ---------- MOTOR INIT ----------
    pinMode(vcc, OUTPUT);
    digitalWrite(vcc, HIGH);

    for (int i = 0; i < 4; i++) {
        pinMode(motors[i], OUTPUT);
    }

    digitalWrite(break_L, HIGH);
    digitalWrite(break_R, HIGH);

    pinMode(speed_L, OUTPUT);
    pinMode(speed_R, OUTPUT);

    analogWrite(speed_L, 0);
    analogWrite(speed_R, 0);

    // ---------- ULTRASONIC INIT ----------
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);

    // ---------- WIFI INIT ----------
    WiFi.beginAP(ssid, pass);
    server.begin();

    Serial.println("Robot Ready.");
}
// 
void stopMotors(){
    analogWrite(speed_R, 0);
    analogWrite(speed_L, 0);  
}
void moveBackwoard(int speed = 70){
    analogWrite(speed_R, speed);
    analogWrite(speed_L, speed);

    digitalWrite(dir_L, HIGH);
    digitalWrite(dir_R, HIGH);

    digitalWrite(break_L, LOW);
    digitalWrite(break_R, LOW);    
}
void moveForward(int speed = 70){
    analogWrite(speed_R, speed);
    analogWrite(speed_L, speed);

    digitalWrite(dir_L, LOW);
    digitalWrite(dir_R, LOW);

    digitalWrite(break_L, LOW);
    digitalWrite(break_R, LOW);    
}
void turnRight(int speed = 70){
    analogWrite(speed_R, speed);
    analogWrite(speed_L, speed);

    digitalWrite(dir_L, LOW);
    digitalWrite(dir_R, HIGH);

    digitalWrite(break_L, LOW);
    digitalWrite(break_R, LOW);
}
void turnLeft(int speed = 70){
    analogWrite(speed_R, speed);
    analogWrite(speed_L, speed);

    digitalWrite(dir_L, HIGH);
    digitalWrite(dir_R, LOW);

    digitalWrite(break_L, LOW);
    digitalWrite(break_R, LOW);
}
// =============== LINE DETECTION ======================
bool isLineDetected() {
     qtr.readLineBlack(sensorValues);

    for (int i = 0; i < sensorCount; i++) {
        if (sensorValues[i] > 500)
            return true;
    }

    return false;
}
// ================= PID CONTROLS =======================
// === LINE ===
void PID_Line() {
    unsigned int position = qtr.readLineBlack(sensorValues);

    error = SETPOINT - position;

    P = error;
    I = I + error;
    D = error - previousError;

    PID_value = (Kp * P) + (Ki * I) + (Kd * D);

    previousError = error;

    analogWrite(speed_R, constrain(SET_SPEED - PID_value, MIN_SPEED, MAX_SPEED));
    analogWrite(speed_L, constrain(SET_SPEED + PID_value, MIN_SPEED, MAX_SPEED));

    digitalWrite(dir_L, LOW);
    digitalWrite(dir_R, LOW);
    digitalWrite(break_L, LOW);
    digitalWrite(break_R, LOW);
}
// === OBJECT ===
int PID_Object() {
  unsigned int dist = sonar.ping();

  if (dist == 0) return 0;

  err_obj = OBJ_SETPOINT - dist;

  P_obj = err_obj;
  I_obj += err_obj;
  D_obj = err_obj - prevErr_obj;

  PID_obj = (Kp_obj * P_obj) + (Ki_obj * I_obj) + (Kd_obj * D_obj);

  prevErr_obj = err_obj;

  analogWrite(speed_L, constrain(SET_SPEED_OBJ + PID_obj, MIN_SPEED_OBJ, MAX_SPEED_OBJ));
  analogWrite(speed_R, constrain(SET_SPEED_OBJ - PID_obj, MIN_SPEED_OBJ, MAX_SPEED_OBJ));

  digitalWrite(dir_L, LOW);
  digitalWrite(dir_R, LOW);
  digitalWrite(break_L, LOW);
  digitalWrite(break_R, LOW);
}
// ================= MAIN LOOP =========================
void loop() {
    // ServoMtr.write(180);
    // delay(100);
    // Serial.println(sonar.ping());
    // delay(1000);

switch (roboState) {
    case followLine:
        PID_Line();
        distance = sonar.ping();
        if (distance < 300 && distance != 0) {
            stopMotors();
            while (sonar.ping() < 300) {
            moveBackwoard();
            }
            while (isLineDetected()) {
            turnRight();
            }
            stopMotors();
            delay(1000);
            while (sonar.ping_cm() < 9) {
            turnRight();
            }
            stopMotors();
            delay(1000);
            ServoMtr.write(135);
            delay(100);
            while (sonar.ping_cm() > 10) {
            moveForward();
            }
            stopMotors();
            delay(1000);
            while (sonar.ping_cm() < 22) {
            turnRight();
            }
            stopMotors();
            delay(1000);
            ServoMtr.write(180);
            delay(100);
            while (sonar.ping_cm() < 24 ) {
            moveForward();
            }
            stopMotors();
            delay(1000);
            roboState = avoidObstacle;
        }
    break;
    case avoidObstacle:
        while (!isLineDetected()) {
            PID_Object();
        }
        stopMotors();
        turnRight();
        delay(200);
        stopMotors();
        ServoMtr.write(90);
        delay(100);
      roboState = followLine;
    break;
  }
}