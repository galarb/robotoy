class Motor {
  private:
    int pwm1;
    int pwm2;

  public:
    Motor(int pin1, int pin2) {
      pwm1 = pin1;
      pwm2 = pin2;
    }

    void begin() {
      pinMode(pwm1, OUTPUT);
      pinMode(pwm2, OUTPUT);
      stop();
    }

    void setSpeed(int speed) {
      speed = constrain(speed, -255, 255);

      if (speed > 0) {
        analogWrite(pwm1, speed);
        analogWrite(pwm2, 0);
      }
      else if (speed < 0) {
        analogWrite(pwm1, 0);
        analogWrite(pwm2, abs(speed));
      }
      else {
        stop();
      }
    }

    void stop() {
      analogWrite(pwm1, 0);
      analogWrite(pwm2, 0);
    }
    void brake(){
      analogWrite(pwm1, 255);
      analogWrite(pwm2, 255); 
    }
};
class Ultrasonic {
  private:
    int trigPin;
    int echoPin;

  public:
    Ultrasonic(int trig, int echo) {
      trigPin = trig;
      echoPin = echo;
    }

    void begin() {
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
      digitalWrite(trigPin, LOW);
    }

    long readCM() {
      // Trigger pulse
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // Read echo (timeout 25ms ≈ 4m)
      long duration = pulseIn(echoPin, HIGH, 25000);

      if (duration == 0) {
        return -1; // no echo
      }

      // Sound speed: 343 m/s
      long distance = duration * 0.034 / 2;
      return distance;
    }
};

class TCRT5000 {
  private:
    int pin;
    int threshold;
    int minVal;
    int maxVal;
  public:
    TCRT5000(int analogPin, int thresh = 500)
      : pin(analogPin),
        threshold(thresh),
        minVal(1023),
        maxVal(0) {}

    void begin() {
      pinMode(pin, INPUT);
    }

    int readRaw() {
      return analogRead(pin);
    }

    void updateCalibration() {
      int val = readRaw();

      if (val < minVal) minVal = val;
      if (val > maxVal) maxVal = val;
    }

    void setCalibration(int white, int black) {
      minVal = white;
      maxVal = black;
    }



    int readNormalized() {
      int raw = readRaw();

      if (maxVal == minVal) return 0;

      float normalized =
        (raw - minVal) * 100.0 / (maxVal - minVal);

      normalized = constrain(normalized, 0, 100);

      return (int)normalized;
    }


    bool isLine() {
      return readNormalized() > 50;
    }

    void setThreshold(int thresh) {
      threshold = thresh;
    }
};

class PID {
  private:
    double kp, ki, kd;
    double lastError = 0;
    double cumError = 0;
    unsigned long previousTime = 0;

  public:
    PID(double p, double i, double d) {
      kp = p;
      ki = i;
      kd = d;
    }

    double compute(double input, double setpoint) {
      unsigned long currentTime = micros();
      double elapsedTime = (currentTime - previousTime) / 1000000.0;
      if (elapsedTime <= 0) return 0;

      double error = setpoint - input;
      // reset integral on sign change
      if (error * lastError < 0) {
        cumError = 0;
      }

      cumError += error * elapsedTime;
      double rateError = (error - lastError) / elapsedTime;

      double out = kp * error + ki * cumError + kd * rateError;

      lastError = error;
      previousTime = currentTime;

      return constrain(out, -254, 254);
    }
};
class IRQs {

  private:
    volatile bool flag1 = false;
    volatile bool flag2 = false;

    volatile unsigned long lastInterruptTime1 = 0;
    volatile unsigned long lastInterruptTime2 = 0;

    const unsigned long debounceTime = 150; // ms

    int _pin1;
    int _pin2;

    static IRQs* instance;

    static void isr1() {
      unsigned long now = millis();
      if (now - instance->lastInterruptTime1 > instance->debounceTime) {
        instance->flag1 = true;
        instance->lastInterruptTime1 = now;
      }
    }

    static void isr2() {
      unsigned long now = millis();
      if (now - instance->lastInterruptTime2 > instance->debounceTime) {
        instance->flag2 = true;
        instance->lastInterruptTime2 = now;
      }
    }

  public:

    IRQs(int pin1, int pin2) {

      instance = this;

      _pin1 = pin1;
      _pin2 = pin2;

      pinMode(_pin1, INPUT_PULLUP);
      pinMode(_pin2, INPUT_PULLUP);

      attachInterrupt(digitalPinToInterrupt(_pin1), isr1, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pin2), isr2, FALLING);
    }

    bool button1() {
      if (flag1) {
        flag1 = false;
        return true;
      }
      return false;
    }

    bool button2() {
      if (flag2) {
        flag2 = false;
        return true;
      }
      return false;
    }
};

IRQs* IRQs::instance = nullptr;

#include <Wire.h>
#include <TM1650.h>
#include "ButtonIRQ.h"
#include <Adafruit_NeoPixel.h>

// -------- Pin definitions --------
// Motor A
const int PWM_L1 = 9;
const int PWM_L2 = 6;

// Motor B
const int PWM_R1 = 10;
const int PWM_R2 = 11;
//Ultrasonic
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;
//reflected light sensor
const int TCRT_PIN = A0;
//******connect the 4Digit to A4/A5 SDA/SCL******
const int ledspin = 4;
const int NUMPIXELS = 2;

int baseSpeed = 40; // for tracking line

char buf[5];   // 4 digits + null
unsigned long lastDisplayUpdate = 0;
const unsigned long displayInterval = 100;  // ms
//button mode
bool showLineSensor = true;   // current display mode
int mode;
enum DisplayMode {
  MODE_LINE,
  MODE_RADAR
};
DisplayMode currentMode = MODE_LINE;

//for 4Digit
int digit = 0;
unsigned long lastStep = 0;
unsigned long startTime;

// Create objects
Motor motorL(PWM_L1, PWM_L2);
Motor motorR(PWM_R1, PWM_R2);
Ultrasonic radar(TRIG_PIN, ECHO_PIN);
TCRT5000 lineSensor(TCRT_PIN, 600);
PID usPID(1.7, 0.7, 0.0);
PID linePID(1.0, 0.0, 0.0);
TM1650 d;
IRQs buttons(2, 3);
Adafruit_NeoPixel pixels(NUMPIXELS, ledspin, NEO_GRB + NEO_KHZ800);



void setup() {
  Serial.begin(115200);
  Wire.begin(); //Join the bus as master
  motorL.begin();
  motorR.begin();
  radar.begin();
  lineSensor.begin();
  d.init();
  d.setBrightness(TM1650_MIN_BRIGHT);
  pixels.begin();
  startTime = millis();
  pixels.clear();
  pixels.setBrightness(40);
}

void loop() {
  //make sure the robot sees both black and white during this caliblration time (3000)!!!
  while (millis() - startTime < 3000) {
    unsigned long now = millis();
    lineSensor.updateCalibration();
    if (now - lastStep >= 150) {
        lastStep = now;
        d.clear();
        d.setDot(digit, 1);
        digit++;
        if (digit > 3) digit = 0;
    }    
  }
  activate();
  displaymode();
  delay(1);

}

void goUS(int sp){
  long dist = radar.readCM();
  if (dist > 0){
    Serial.println(dist);
    int speed = -usPID.compute(dist, sp);
    Serial.print("speed ="); Serial.println(speed);
    motorL.setSpeed(speed);
    motorR.setSpeed(speed);
  }
  
}
void trackLine(int sp){
  int reflectedlight = lineSensor.readRaw();
  
  int speed = linePID.compute(reflectedlight, sp);
  motorR.setSpeed(baseSpeed+speed);
  motorL.setSpeed(baseSpeed-speed);
}

void displaymode() {

  unsigned long now = millis();

  if (buttons.button1()) {
    currentMode = (currentMode == MODE_LINE)
                  ? MODE_RADAR
                  : MODE_LINE;
  }

  if (now - lastDisplayUpdate < displayInterval)
    return;

  lastDisplayUpdate = now;

  if (currentMode == MODE_LINE) {

    sprintf(buf, "%4d", lineSensor.readNormalized());

  } else {

    long dist = radar.readCM();
    if (dist <= 0) return;

    sprintf(buf, "%4d", dist);
  }

  d.displayString(buf);

  // 🔥 ALWAYS reapply dot
  d.setDot(0, currentMode == MODE_RADAR);
}



void activate() {
  if (buttons.button2()) {
    //Serial.println("LButton pressed");
    mode++;   // increment mode

    if (mode > 3) {
      mode = 1;
    }
  }
  switch (mode) {

    case 1:
      //Serial.println("idle mode");
      pixels.setBrightness(10);
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
      pixels.setPixelColor(1, pixels.Color(0, 255, 0));
      pixels.show();
      motorR.stop();
      motorL.stop();
      break;

    case 2:
      Serial.println("track mode");
      trackLine(50);
      pixels.setBrightness(100);
      pixels.setPixelColor(0, pixels.Color(0, 0, 255));
      pixels.setPixelColor(1, pixels.Color(0, 0, 255));
      pixels.show();
      break;

    case 3:
      Serial.println("US mode");
      goUS(30);
      pixels.setBrightness(40);

      pixels.setPixelColor(0, pixels.Color(0, 255, 100));
      pixels.setPixelColor(1, pixels.Color(0, 255, 100));
      pixels.show();
      break;

  }
}
