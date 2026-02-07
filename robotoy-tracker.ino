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

  public:
    TCRT5000(int analogPin, int thresh = 500) {
      pin = analogPin;
      threshold = thresh;
    }

    void begin() {
      pinMode(pin, INPUT);
    }

    int readRaw() {
      return analogRead(pin);
    }

    bool isLine() {
      return readRaw() > threshold;
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


// -------- Pin definitions --------
// Motor A
const int PWM_L1 = 2;
const int PWM_L2 = 3;

// Motor B
const int PWM_R1 = 10;
const int PWM_R2 = 11;
//Ultrasonic
const int TRIG_PIN = 13;
const int ECHO_PIN = 12;
//reflected light sensor
const int TCRT_PIN = A0;
int baseSpeed = 40; // for tracking line
// Create objects
Motor motorL(PWM_L1, PWM_L2);
Motor motorR(PWM_R1, PWM_R2);
Ultrasonic radar(TRIG_PIN, ECHO_PIN);
TCRT5000 lineSensor(TCRT_PIN, 600);
PID usPID(1.0, 0.0, 0.0);
PID linePID(1.0, 0.0, 0.0);

void setup() {
  motorL.begin();
  motorR.begin();
  radar.begin();
  lineSensor.begin();

  Serial.begin(115200);
  

}

void loop() {
  goUS(10);
}

void goUS(int sp){
  long dist = radar.readCM();
  Serial.println(dist);
  int speed = -usPID.compute(dist, sp);
        Serial.print("speed ="); Serial.println(speed);
  motorL.setSpeed(speed);
  motorR.setSpeed(speed);
}
void trackLine(int sp){
  int reflectedlight = lineSensor.readRaw();
  int speed = linePID.compute(reflectedlight, sp);
  motorR.setSpeed(baseSpeed+speed);
  motorL.setSpeed(baseSpeed-speed);
}
