// #define ENABLE_SERIAL_OUTPUT_VERBOSE  //uncomment this to enable more detailed logs.
// #define ENABLE_SERIAL_OUTPUT_SIMPLE

#include <TinyStepper.h>

// Define GPIO pinout to the ULN2003 Darlington Array to drive a 28BYJ-48 Stepper Motor
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5
#define HALFSTEPS 4096 // Number of half-steps for a full rotation
// Initialize the TinyStepper Class
TinyStepper stepper(HALFSTEPS, IN1, IN2, IN3, IN4);

// Define GPIO pinout to the K93 Hall Effect Sensor
#define HALL_SENSOR_PIN A0
#define CLOSED_POSITION_THRESHOLD 700
float hallSensorVoltage;
bool isBaffleClosed = false;

// Define GPIO pinout to the push button
#define BUTTON_PIN 6
int buttonState;

void setup()
{
  Serial.begin(115200);
  stepper.Enable(); // Enable the stepper motor
  delay(500);
  pinMode(HALL_SENSOR_PIN, INPUT); // Set the hall effect sensor pin as an input (analog)
  delay(500);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // initial reading of baffle position on start-up
  if (analogRead(HALL_SENSOR_PIN) < CLOSED_POSITION_THRESHOLD)
  {
    isBaffleClosed = false;
  }

} //  END OF SETUP

void loop()
{
#ifdef ENABLE_SERIAL_OUTPUT_VERBOSE
  Serial.print("scanning Button on pin: ");
  Serial.println(BUTTON_PIN);
#endif
  buttonState = digitalRead(BUTTON_PIN); // take a reading of the button state
#ifdef ENABLE_SERIAL_OUTPUT_VERBOSE
  Serial.print("Button on pin ");
  Serial.print(BUTTON_PIN);
  Serial.print(" current state: ");
  Serial.println(buttonState);
#endif

  if (buttonState == LOW)
  { // if he button is pressed
#ifdef ENABLE_SERIAL_OUTPUT_VERBOSE
    //  print current button state
    Serial.print("Button on pin ");
    Serial.print(BUTTON_PIN);
    Serial.println(" was pressed");
    // print current baffle state
    Serial.print("is baffle closed?");
    Serial.println(isBaffleClosed);
#endif
    if (isBaffleClosed == false)
    { //  if the baffle is currently open
#ifdef ENABLE_SERIAL_OUTPUT_SIMPLE
      Serial.println("Baffle currently open, attempting to close.");
#endif
      CloseBaffle(); // try to close the baffle until it reaches closed position
    }
    else if (isBaffleClosed == true)
    {
#ifdef ENABLE_SERIAL_OUTPUT_SIMPLE
      Serial.println("Baffle currently closed, attempting to open.");
#endif
      OpenBaffle();
    }
  }
} // END OF MAIN LOOP

void CloseBaffle()
{
#ifdef ENABLE_SERIAL_OUTPUT_SIMPLE
  Serial.println("Closing Baffle...");
#endif
  while (analogRead(HALL_SENSOR_PIN) < CLOSED_POSITION_THRESHOLD) // while the baffle is in the open position
  {
    isBaffleClosed = false; // set the baffle position flag state to open
    stepper.Move(-1);       // rotate the baffle -1 degree (backward)
    delay(10);              // Pause to make sure motor gets there
    if (analogRead(HALL_SENSOR_PIN) > CLOSED_POSITION_THRESHOLD)
    { // sub loop in the while loop
      isBaffleClosed = true;
#ifdef ENABLE_SERIAL_OUTPUT_SIMPLE
      Serial.println(" ");
      Serial.println("#Baffle closed#");
#endif
      break; // break out of loops if the baffle is closed
    }
#ifdef ENABLE_SERIAL_OUTPUT_SIMPLE
    Serial.print(".");
#endif
  }
}

void OpenBaffle()
{
  if (analogRead(HALL_SENSOR_PIN) > CLOSED_POSITION_THRESHOLD) // if baffle is currently in the close position
  {
#ifdef ENABLE_SERIAL_OUTPUT_SIMPLE
    Serial.println("Opening baffle.");
#endif
    // stepper.AccelMove(90); // move the baffle 90 degrees to the fully open position (forward)
    stepper.AccelMove(90, 1, 10);
    isBaffleClosed = false; // set the baffle position flag state to close
#ifdef ENABLE_SERIAL_OUTPUT_SIMPLE
    Serial.println("Baffle opened");
#endif
  }
}

float ReadButton()
{
  buttonState = digitalRead(BUTTON_PIN); // take a reading from the button
  return buttonState;                    // return the value of the reading
}

float ReadHallSensor()
{
  hallSensorVoltage = analogRead(HALL_SENSOR_PIN); // Take a reading from the hall effect sensor
  return hallSensorVoltage;                        // Return the value of the reading
}