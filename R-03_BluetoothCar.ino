#include <SoftwareSerial.h>
#include <NewPing.h>

int bluetoothTx = 7;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 8;  // RX-I pin of bluetooth mate, Arduino D3

int dataFromBt;
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

// Right Motor
int eR = 11;
int inR1 = 10;
int inR2 = 9;

// Left Motor
int eL = 3;
int inL1 = 5;
int inL2 = 6;

// UltraSonic Sensor
#define TRIGGER_PIN  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

#define MAX_SPEED 255
#define DANGER_DISTANCE 20 // cm

// led
int ledPin = 12;

// speed (case '0' at the beginning)
int speed = MAX_SPEED - 50;

void setup()
{
  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.begin(9600);  // Start bluetooth serial at 9600

  // Hardware serial port for debugging
  // signals from smart phone
  Serial.begin(9600);

  pinMode(eR, OUTPUT);
  pinMode(inR1, OUTPUT);
  pinMode(inR2, OUTPUT);

  pinMode(eL, OUTPUT);
  pinMode(inL1, OUTPUT);
  pinMode(inL2, OUTPUT);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
}

void loop()
{
  int distance = sonar.ping_cm();

  // If the bluetooth sent any characters
  if (bluetooth.available()) {
    dataFromBt = bluetooth.read();

    // Log out the signals from smartphone
    Serial.println((char) dataFromBt);

    switch(dataFromBt) {
      // foward
      case 'F': case 'f':
        if(distance <= DANGER_DISTANCE) {
          setMotorR(0, 0);
          setMotorL(0, 0);
        } else {
          setMotorR(speed, 0);
          setMotorL(speed, 0);
        }
        // setMotorR(speed, 0);
        // setMotorL(speed, 0);
        break;
      // backward
      case 'B': case 'b':
        setMotorR(speed, 1);
        setMotorL(speed, 1);
        break;
      // left
      case 'L': case 'l':
        setMotorR(speed, 0);
        setMotorL(0, 0);
        break;
      // right
      case 'R': case 'r':
        setMotorR(0, 0);
        setMotorL(speed, 0);
        break;
      // stop
      case 'S': case 's':
        setMotorR(0, 0);
        setMotorL(0, 0);
        break;
      // velocity
      case '1': case '2': case '3': case '4': case '5':
      case '6': case '7': case '8': case '9': case '0':
        speed = MAX_SPEED - (10 - dataFromBt) * 5;
        break;
      case 'q':
        speed = MAX_SPEED;
        break;
    }
  }
}

void setMotorR(int speed, boolean reverse)
{
  analogWrite(eR, speed);
  digitalWrite(inR1, !reverse);
  digitalWrite(inR2, reverse);
}

void setMotorL(int speed, boolean reverse)
{
  analogWrite(eL, speed);
  digitalWrite(inL1, !reverse);
  digitalWrite(inL2, reverse);
}
