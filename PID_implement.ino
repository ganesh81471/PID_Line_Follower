// Define pins and constants
const int sensor1Pin = A0;
const int sensor2Pin = A1;
const int sensor3Pin = A2;
const int motorRPWM = 2;
const int motorLPWM = 11;
const int motorR1 = 4;
const int motorR2 = 5;
const int motorL1 = 9;
const int motorL2 = 10;

// PID constants (adjust as needed)
double Kp = 10.0;
double Ki = 0.00001;
double Kd = 0.6;

// Variables for PID calculations and sensor readings
double Input, Output, Setpoint, integral, previousError;
int sensorValues[3];
unsigned long lastTime;

void setup() {
  // Initialize serial communication and pins
  Serial.begin(9600);
  Serial.println("Setup started");  // Debugging statement

  // Pin setup
  pinMode(sensor1Pin, INPUT);
  pinMode(sensor2Pin, INPUT);
  pinMode(sensor3Pin, INPUT);
  pinMode(motorRPWM, OUTPUT);
  pinMode(motorLPWM, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);

  lastTime = millis();
  Setpoint = 10;  // Adjust setpoint if needed

  Serial.println("Setup complete");  // Debugging statement
}

void loop() {
  // Read sensor values
  sensorValues[0] = analogRead(sensor1Pin);
  sensorValues[1] = analogRead(sensor2Pin);
  sensorValues[2] = analogRead(sensor3Pin);

  // Print sensor readings for debugging
  Serial.print("Sensor values: ");
  Serial.print(sensorValues[0]);
  Serial.print(", ");
  Serial.print(sensorValues[1]);
  Serial.print(", ");
  Serial.println(sensorValues[2]);

  // Error checking for invalid sensor readings
  if (sensorValues[0] < 0 || sensorValues[1] < 0 || sensorValues[2] < 0) {
    Serial.println("Invalid sensor readings! Check connections and calibration.");
    // Implement error handling here (e.g., stop motors, blink LED)
    // You may want to add a delay or return to prevent continuous motor control
   delay(1000);
    return;
  }

  // Calculate error and PID terms
  Input = sensorValues[0] - sensorValues[2];  // Adjust error calculation if needed

  unsigned long now = millis();
  double elapsedTime = (now - lastTime) / 1000.0;
  lastTime = now;

  double error = Input - Setpoint;
  integral += error * elapsedTime;
  double derivative = (error - previousError) / elapsedTime;
  previousError = error;

  Output = Kp * error + Ki * integral + Kd * derivative;

  // Print PID values for debugging
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(", Output: ");
  Serial.println(abs(Output));
  

  // Apply output limits
  Output = constrain(Output, -255, 255);

  // Motor control
  Serial.println("Setting motor directions and speeds");  // Debugging statement
  digitalWrite(motorR1, Output > 0 ? HIGH : LOW);
  digitalWrite(motorR2, Output > 0 ? LOW : HIGH);
  digitalWrite(motorL1, Output < 0 ? HIGH : LOW);
  digitalWrite(motorL2, Output < 0 ? LOW : HIGH);
  analogWrite(motorRPWM, abs(Output) + 105);
  analogWrite(motorLPWM, abs(Output) - 105);

  // Introduce a delay to control the loop rate
  delay(10);
}
