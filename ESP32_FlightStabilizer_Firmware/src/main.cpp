// =======================================================
//                    INCLUDED LIBRARIES
// =======================================================
#include <Wire.h>       // For I2C communication with MPU6050 
#include <WiFi.h>       // For connecting ESP32 to WiFi 
#include <WiFiClient.h> // For handling WiFi clients (like a phone or computer) 
#include <WiFiServer.h> // For setting up a TCP server on ESP32 
#include <ESPmDNS.h>    // For mDNS (allows accessing ESP32 via name like esp32.local) 
#include <EEPROM.h>     // For saving configuration data across reboots 
#include <ESP32Servo.h> // To control servo motors on ESP32 
#include <stdlib.h>     // for memory allocation related tasks  
#include <string.h>      
#include "freertos/FreeRTOS.h" //primary interface for application code to interact with the RTOS kernel
#include "freertos/semphr.h"   //provides functions and macros for creating and managing semaphores  

// =======================================================
//                  CONSTANTS & CONFIGURATION
// =======================================================

// ---------------- EEPROM Configuration ----------------
// Defines how much space EEPROM will use and where to store parameter
#define EEPROM_SIZE       36
#define ACCEL_FILTER_ADDR 0
#define GYRO_FILTER_ADDR  4
#define COMP_FILTER_ADDR  8
#define PITCH_PID_ADDR   12
#define ROLL_PID_ADDR    24

#define M_PI 3.14159265358979323846

// ---------------- MPU6050 Settings ---------------------
// gSensitivity is for converting raw gyro data into degrees/sec (depends on gyro config)
#define MPU6050_I2C_ADDRESS 0x68
float FREQ = 50.0;
double gSensitivity = 65.5; 

// ---------------- PWM Input Pins -----------------------
#define PITCH_IP      15 //rx2
#define ROLL_IP      16 //d15
#define YAW_IP       17 //null
#define AUTO_PILOT   18 //d18

// PWM pulse width boundaries (microseconds) used for signal normalization
#define MIN_PULSE_WIDTH 999
#define MAX_PULSE_WIDTH 1993
#define PULSE_TIMEOUT   25000 //Maximum time to wait for a pulse, it prevents code blocking when signal lost

// ---------------- Servo Output Pins --------------------
#define PITCH_SERVO_PIN 25
#define ROLL_SERVO_PIN  26

// ---------------- WiFi Settings -------------------------
const char* ssid     = "aju";
const char* password = "@ajujcd@";
WiFiServer server(12345); 

// ---------------- LED Pin ------------------------------
const int ledPin = 2;

// --- Network Buffer Configuration ---
#define MAX_COMMAND_LEN 30

// =======================================================
//                    GLOBAL VARIABLES
// =======================================================

// --- Mutex for protecting shared configuration data ---
SemaphoreHandle_t configMutex; 

// ---------------- Filter and State ---------------------
// Filter coefficients control responsiveness vs. smoothness of sensor data.
float ACCEL_FILTER   = 0.3;
float GYRO_FILTER    = 0.08;
float COMP_FILTER    = 0.7;

// Tracks stabilization references and mode states
float referencePitch = 0;
float referenceRoll  = 0;
float pitchOffset    = 90; // servo center = 90
float rollOffset     = 90;
bool firstStabLoop   = true; // Used to set reference angles on first stabilization loop after mode switch
bool lastStabState   = false;// Tracks the last known stabilization mode to detect changes and set reference angles accordingly.
// Global flags to communicate streaming mode between tasks
volatile bool cubeStreaming = false; 
volatile bool pwmStreaming = false;
bool inside          = false; // Used for LED blinking pattern to indicate stabilization mode (blinks when inside stabilization mode)

// ---------------- IMU Data -----------------------------
// Raw and filtered accelerometer & gyroscope values.
// Offsets calculated during calibration.
// gx, gy, gz: estimated angles('volatile' for multi-core read/write safety)
volatile double gx = 0, gy = 0, gz = 0; // These represent the current estimated orientation angles (in degrees) of the device, calculated by integrating gyroscope data and applying a complementary filter with accelerometer data. They are updated in real-time as new sensor data is read and processed in the updateMPU6050() function. Marked as 'volatile' because they are accessed from both the main control loop (Core 1) and the network communication task (Core 0), ensuring that changes made in one core are immediately visible to the other without caching issues.
double gyrX = 0, gyrY = 0, gyrZ = 0; // Raw gyroscope readings in degrees per second, after bias correction and scaling. These represent the current angular velocity around each axis and are used for both control calculations and streaming to clients. They are updated every time new sensor data is read in the updateMPU6050() function.
double gyrXoffs = 0, gyrYoffs = 0, gyrZoffs = 0; // Gyroscope offsets (biases) calculated during calibration. These values are subtracted from raw gyroscope readings to correct for any constant bias in the sensor, ensuring more accurate angle estimation. They are determined by taking multiple readings while the device is stationary and averaging them to find the inherent bias in each axis.

int16_t accX = 0, accY = 0, accZ = 0; // Raw accelerometer readings in the X, Y, and Z axes. These values represent the current acceleration experienced by the device along each axis, including the effect of gravity. They are used to estimate the device's orientation (pitch and roll) and are updated every time new sensor data is read in the updateMPU6050() function.

double filtered_ax = 0, filtered_ay = 0, filtered_az = 0; // Low-pass filtered accelerometer values. These smoothed values are used to calculate the device's orientation angles (pitch and roll) with reduced noise, providing more stable control inputs. They are updated in the updateMPU6050() function using a simple exponential moving average filter based on the raw accelerometer readings.
double filtered_gx = 0, filtered_gy = 0, filtered_gz = 0; // Low-pass filtered gyroscope values. These smoothed angular velocity readings are used in the complementary filter to estimate the device's orientation angles (gx, gy, gz) with reduced noise, improving stability and control performance. They are updated in the updateMPU6050() function using a simple exponential moving average filter based on the raw gyroscope readings.

unsigned long lastMPUTime = 0;

// ---------------- PID Control --------------------------
// PID structure to hold gains and error history for pitch & roll.
struct PID {
  float Kp;
  float Ki;
  float Kd;
  float integral;
  float previous_error;
  unsigned long last_time;
}; // This struct defines the parameters and state variables for a PID controller. Kp, Ki, and Kd are the proportional, integral, and derivative gains that determine how the controller responds to errors. The integral variable accumulates the error over time to eliminate steady-state error, while previous_error is used to calculate the derivative term, which predicts future error based on its rate of change. last_time is used to calculate the time difference between updates, ensuring that the PID calculations are based on accurate timing.(refer: What is a PID Controller? | DigiKey(youtube) for more information)

PID pitchPID = {2.0, 0.1, 0.5, 0.0, 0.0, 0}; // Initializes the PID controller for pitch with specific gains (Kp=2.0, Ki=0.1, Kd=0.5) and zeroed integral and error history. These values can be tuned to achieve the desired responsiveness and stability in the pitch control of the flight stabilizer. The last_time is initialized to 0 and will be set to the current time when the PID controller is first used to ensure proper timing for subsequent calculations.
PID rollPID  = {2.0, 0.1, 0.5, 0.0, 0.0, 0}; // Initializes the PID controller for roll with the same gains and initial state as the pitch controller. This allows for independent tuning of the roll control while maintaining a similar response profile to the pitch control. The integral and previous_error are also initialized to 0, and last_time will be set when the controller is first used, just like the pitchPID.

// ---------------- Servo & Input ------------------------
// Servo objects for control
// Stores last known PWM input values for change detection
Servo pitchServo;
Servo rollServo;

int lastPercentage1 = -1, lastPercentage2 = -1; // These variables store the last known percentage values for the PWM inputs corresponding to pitch and roll. They are used to detect changes in the input signals, allowing the system to only send updates to connected clients when there is a change in the input or after a certain time interval. This helps reduce unnecessary network traffic while still providing timely updates when the control inputs are adjusted.
int lastPercentage3 = -1, lastPercentage4 = -1; // Similar to the above, these variables store the last known percentage values for the YAW and AUTO_PILOT PWM inputs. They are used in the same way to detect changes and manage network updates for those specific control inputs, ensuring efficient communication with clients while maintaining responsiveness to user adjustments.

// =======================================================
//                FUNCTION PROTOTYPES
// =======================================================
void loadParameters(); // Reads saved filter and PID values from EEPROM and stores into variables. Also validates them to avoid corrupt values
void saveParameters(); // Writes current filter and PID values to EEPROM for persistence across reboots
void calibrateMPU6050(); // Reads 500 samples from MPU6050 gyro to calculate offsets (bias correction). These offsets are subtracted from all future readings.
void initMPU6050(); // Configures the MPU6050: Wakes it up from sleep mode, Sets sample rate divider, Configures filters and sensitivity
void updateMPU6050(); // Reads MPU6050 data, applies filters, computes orientation angles (gx, gy, gz) and updates global variables. This function is called in the main loop to ensure that the latest sensor data is always available for both control calculations and streaming to clients. It handles the raw I2C communication with the MPU6050, processes the sensor data, and applies the necessary filtering to provide stable and accurate orientation estimates for the flight stabilizer.
void updateServoFromMPU(); // Computes PID outputs based on the current orientation (gx, gy) and reference angles, then updates the servo positions accordingly. This function is called when stabilization mode is active to continuously adjust the servo angles in response to changes in the device's orientation, helping to maintain stable flight. It uses the computePID() function to calculate the necessary adjustments for both pitch and roll based on the current sensor readings and the desired reference angles.

// New RTOS Task Function
void communicationTask(void *pvParameters);  // This function will run on Core 0 and handle all network communication with clients. It listens for incoming TCP connections, processes commands to get/set configuration parameters, and manages streaming of PWM input values and IMU data to connected clients. By running this task on a separate core, we ensure that network communication does not interfere with the real-time control loop running on Core 1, allowing for responsive control and efficient handling of client requests simultaneously.

int i2c_read(int addr, int start, uint8_t* buffer, int size); // This function performs an I2C read operation from the specified device address (addr) starting at the given register (start). It reads 'size' bytes of data into the provided buffer. This is used for reading sensor data from the MPU6050, such as accelerometer and gyroscope readings, by communicating over the I2C bus. The function returns 0 on success and a non-zero value on failure, allowing for error handling in the calling code.
int i2c_write_reg(int addr, int reg, uint8_t data); // This function performs an I2C write operation to the specified device address (addr) and register (reg) with the given data byte. It is used to configure the MPU6050 by writing to its internal registers, such as setting the sample rate, configuring filters, or adjusting sensitivity settings. The function returns 0 on success and a non-zero value on failure, allowing for error handling in the calling code when initializing or configuring the sensor.

float computePID(PID& pid, float setpoint, float input, float currentOutput = 0); // This function computes the PID output based on the provided setpoint (desired value), input (current value), and the current output of the system. It uses the PID structure to access the gains and error history, calculates the proportional, integral, and derivative components, and returns the total PID output. The optional currentOutput parameter allows for bumpless transfer when changing control modes, ensuring that the transition is smooth without causing sudden jumps in the output. This function is called in the updateServoFromMPU() function to calculate the necessary adjustments for servo control based on the current orientation and reference angles.
void resetPID(PID &pid); // This function resets the integral and previous error of the given PID controller to zero and updates the last_time to the current time. It is used to prevent unwanted behavior, such as overshooting due to accumulated error, when control modes change or conditions reset. By calling this function when switching between manual and stabilization modes, we ensure that the PID controller starts fresh without any residual error from the previous mode, allowing for smoother transitions and more stable control performance.

// =======================================================
//                      SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nStarting Flight Controller...");

  pinMode(ledPin, OUTPUT);

  // Initialize Mutex for configuration parameters
  configMutex = xSemaphoreCreateMutex(); // Creates a mutex to protect access to shared configuration parameters (filter coefficients and PID gains) between the main control loop (Core 1) and the network communication task (Core 0). This ensures that when one core is reading or writing these parameters, the other core cannot access them simultaneously, preventing data corruption and ensuring thread safety. If the mutex cannot be created, the code will print an error message and halt indefinitely, as safe operation cannot be guaranteed without it.
  if (configMutex == NULL) {
      Serial.println("FATAL: Failed to create configuration mutex.");
      while(true);//Mutex is not created, code stops forever
  }

  // Initialize Hardware
  EEPROM.begin(EEPROM_SIZE); // must be called before using EEPROM.read or .write or . commit
  Wire.begin(21, 22);// SDA = 21, SCL = 22, to start I2C communication with IMU
  
  // Load initial configuration
  loadParameters();//Reads saved filter and PID values from EEPROM and stores into variables. Also validates them to avoid corrupt values
  initMPU6050();//Configures the MPU6050: Wakes it up from sleep mode, Sets sample rate divider, Configures filters and sensitivity
  calibrateMPU6050(); //Reads 500 samples from MPU6050 gyro to calculate offsets (bias correction). These offsets are subtracted from all future readings.

  // Configure I/O
  pinMode(PITCH_IP, INPUT);
  pinMode(ROLL_IP, INPUT);
  pinMode(YAW_IP, INPUT);
  pinMode(AUTO_PILOT, INPUT);

  // Connects the Servo library to the physical pins for servo control.
  pitchServo.attach(PITCH_SERVO_PIN); //ESP32Servo library starts PWM generation on that GPIO pin
  rollServo.attach(ROLL_SERVO_PIN);
  pitchServo.write(pitchOffset); 
  rollServo.write(rollOffset);

  //Take initial sensor reading, Reads MPU6050 data once and sets initial reference angles for stabilization.
  updateMPU6050();
  referencePitch = gy;//Sets the initial reference pitch angle to the current estimated pitch (gy) from the MPU6050. This means that when stabilization mode is activated, the system will try to maintain this initial pitch angle as the target, allowing for stable flight based on the device's orientation at the time of activation.
  referenceRoll = gx;
  
  // ---------------- WiFi & Task Setup (Core 0) ----------------
  // Tries to connect to WiFi in station mode. If it doesn’t connect in 10 seconds, moves on.
  WiFi.mode(WIFI_STA); //The ESP32 acts like a client, connecting to an existing Wi-Fi network
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  unsigned long startAttemptTime = millis();//captures the start time of the connection attempt using millis()(returns time since boot in ms).
  const unsigned long wifiTimeout = 10000; // sets a 10-second timeout

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) { 
    Serial.print(".");
    delay(500); 
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());//returns the local IP address from Wi-Fi router

    if (MDNS.begin("esp32")) { //Starts mDNS (so you can connect via esp32.local).
      Serial.println("mDNS responder started: esp32.local");
    }

    server.begin(); 
    Serial.println("TCP server started on port 12345");
    
    // --- CREATE and PIN Network Task to Core 0 ---
    xTaskCreatePinnedToCore(    // Creates a new FreeRTOS task that runs the communicationTask function on Core 0. This task will handle all network communication with clients, allowing it to run independently of the main control loop on Core 1. By pinning it to Core 0, we ensure that it does not interfere with the real-time control tasks running on Core 1, providing responsive network handling while maintaining stable flight control.
        communicationTask,      // Task function
        "NetworkTask",          // Task name
        10000,                  // Stack size (bytes)
        NULL,                   // Parameter to pass to function (none)
        1,                      // Task priority (lower than default loop())
        NULL,                   // Task handle (unused)
        0                       // Core ID (Core 0)
    );
    Serial.println("Network Task started on Core 0.");
  } else {
    Serial.println("\nWiFi connection failed. Running only flight control loop.");
  }
}

// =======================================================
//                       MAIN LOOP (Core 1)
// =======================================================
void loop() {
  // --- 1. Core Flight Control Loop (runs at FREQ rate on Core 1) ---
  if (millis() - lastMPUTime >= (1000 / FREQ)) {//This block limits how often inputs and IMU data are read — eg FREQ=50 would mean every 20ms.
    lastMPUTime = millis();
    
    // *** CRITICAL FIX: Update IMU data regardless of stabilization mode ***
    // This ensures gx, gy, gz are always fresh for streaming (Core 0) and control (Core 1).
    updateMPU6050(); 
    
    // Read PWM inputs with deadband
    uint32_t pulseWidth1 = pulseIn(PITCH_IP, HIGH, PULSE_TIMEOUT);//Reads the duration of a HIGH pulse on the specified pin (PITCH_IP) with a timeout. This is used to measure the PWM signal from the remote controller for the pitch input. The pulse width is expected to be between MIN_PULSE_WIDTH and MAX_PULSE_WIDTH microseconds, which will then be mapped to a percentage value for control purposes. If no pulse is detected within the timeout period, it returns 0, allowing the code to handle signal loss gracefully.
    uint32_t pulseWidth2 = pulseIn(ROLL_IP, HIGH, PULSE_TIMEOUT);
    uint32_t pulseWidth3 = pulseIn(YAW_IP, HIGH, PULSE_TIMEOUT);
    uint32_t pulseWidth4 = pulseIn(AUTO_PILOT, HIGH, PULSE_TIMEOUT);

    // Each PWM value is mapped from microseconds(1000–2000µs) to 0–100%. Then constrained to stay in that range.
    int p1 = constrain(map(pulseWidth1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
    int p2 = constrain(map(pulseWidth2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
    int p3 = constrain(map(pulseWidth3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
    int p4 = constrain(map(pulseWidth4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);

    // Add deadband around center(3% tolerance), If input is near center(e.g. 48–52),treat it as exactly center to prevent jittering from small fluctuations.
    const int DEADBAND = 3;
    if (abs(p1 - 50) < DEADBAND) p1 = 50;
    if (abs(p2 - 50) < DEADBAND) p2 = 50;
    if (abs(p3 - 50) < DEADBAND) p3 = 50;

    bool isStabilizationActive = (p4 > 90);//Mode Determination(>90)means stabilization mode
    int manualRollAngle = map(p1, 0, 100, 45, 135);
    int manualPitchAngle = map(p2, 0, 100, 45, 135);

    // Handle mode transitions
    // This block detects when the stabilization mode changes (from off to on or on to off) and sets reference angles and resets PID controllers accordingly. 
    // When stabilization is activated, it captures the current orientation as the reference and saves the current servo positions as offsets to ensure a smooth transition without sudden jumps in servo angles. 
    // It also resets the PID controllers to prevent any accumulated error from affecting the new mode. When stabilization is deactivated, it simply prints a message, allowing manual control to take over without any additional adjustments.
    if (isStabilizationActive != lastStabState) {
      if (isStabilizationActive) {
        // Set reference angles
        // gx, gy are already updated above
        referencePitch = gy;
        referenceRoll = gx;
        
        // Save current servo positions as offset
        pitchOffset = pitchServo.read();
        rollOffset = rollServo.read();
        
        // Reset PID controllers to prevent jumps due to accumulated error
        resetPID(pitchPID);
        resetPID(rollPID);
        
        firstStabLoop = false;
        
        Serial.println("Stabilization ON");
      } else {
        Serial.println("Stabilization OFF");
      }
      lastStabState = isStabilizationActive;
    }

    // Apply control based on mode
    if (isStabilizationActive) {
      if (abs(p1 - 50) <= 5 && abs(p2 - 50) <= 5) {
        // Both sticks centered – fully stabilized
        updateServoFromMPU();
      } else {
        // Manual override for non-centered axes
        if (abs(p1 - 50) > 5) {
          resetPID(rollPID);
          rollServo.write(manualRollAngle);
        }
        if (abs(p2 - 50) > 5) {
          resetPID(pitchPID);
          pitchServo.write(manualPitchAngle);
        }
      }
    } else {
      // Manual mode – both axes
      resetPID(rollPID);
      resetPID(pitchPID);
      rollServo.write(manualRollAngle);
      pitchServo.write(manualPitchAngle);
    }

    lastPercentage1 = p1;
    lastPercentage2 = p2;
    lastPercentage3 = p3;
    lastPercentage4 = p4;
  }
  delay(1); 
}

// =======================================================
//              NETWORK HANDLING TASK (Core 0)
// =======================================================
void communicationTask(void *pvParameters) {
  // This task runs on Core 0 and handles all network communication with clients. It listens for incoming TCP connections, processes commands to get/set configuration parameters, and manages streaming of PWM input values and IMU data to connected clients. By running this task on a separate core, we ensure that network communication does not interfere with the real-time control loop running on Core 1, allowing for responsive control and efficient handling of client requests simultaneously.
  for (;;) { 
    WiFiClient client = server.available();//Waits for a client to connect to the TCP server. When a client connects, it returns a WiFiClient object that can be used to communicate with that client. If no client is connected, it returns an empty client object. This allows the task to handle multiple clients sequentially, processing commands and streaming data as needed while ensuring that the main control loop on Core 1 remains responsive and unaffected by network communication.

    if (client) {
      Serial.println("Client connected on Core 0");
      client.setTimeout(2);//Sets a timeout of 2 seconds for client operations. This means that if the client does not send any data or respond within 2 seconds, the operations will fail, allowing the server to handle unresponsive clients gracefully without blocking indefinitely. This is important for maintaining the responsiveness of the network communication task and ensuring that it can continue to serve other clients or perform necessary cleanup when a client becomes unresponsive.

      char command[MAX_COMMAND_LEN + 1];
      char responseBuffer[60]; 

      pwmStreaming = false;
      cubeStreaming = false;
      unsigned long lastSent = 0;

      while (client.connected()) {
        
        if (!client.connected() && client.available() == 0) break;
        
        if (client.available()) {//Checks if the client has sent any data. If data is available, it proceeds to read the command from the client. This allows the server to respond to client requests in real-time while maintaining the connection as long as the client is active. If no data is available, it continues to check for incoming data or handle streaming as needed without blocking the communication task.
          size_t bytesRead = client.readBytesUntil('\n', command, MAX_COMMAND_LEN);
          command[bytesRead] = '\0'; 

          if (bytesRead > 0 && command[bytesRead - 1] == '\r') {
              command[bytesRead - 1] = '\0';
          }
          
          Serial.printf("Received on Core 0: %s\n", command);

          if (xSemaphoreTake(configMutex, (TickType_t)10) == pdTRUE) { 
              
              if (strcmp(command, "get") == 0) {
                snprintf(responseBuffer, sizeof(responseBuffer), "%.3f,%.3f,%.3f\n", ACCEL_FILTER, GYRO_FILTER, COMP_FILTER);
                client.print(responseBuffer);

              } else if (strcmp(command, "getPitchPID") == 0) {
                snprintf(responseBuffer, sizeof(responseBuffer), "%.3f,%.3f,%.3f\n", pitchPID.Kp, pitchPID.Ki, pitchPID.Kd);
                client.print(responseBuffer);
                
              } else if (strcmp(command, "getRollPID") == 0) {
                snprintf(responseBuffer, sizeof(responseBuffer), "%.3f,%.3f,%.3f\n", rollPID.Kp, rollPID.Ki, rollPID.Kd);
                client.print(responseBuffer);

              } else if (strncmp(command, "setA", 4) == 0 && command[4] != '\0') {
                ACCEL_FILTER = atof(command + 4);
                client.println("OK");
              } else if (strncmp(command, "setG", 4) == 0 && command[4] != '\0') {
                GYRO_FILTER = atof(command + 4);
                client.println("OK");
              } else if (strncmp(command, "setC", 4) == 0 && command[4] != '\0') {
                COMP_FILTER = atof(command + 4);
                client.println("OK");
              } else if (strncmp(command, "setPitchP", 9) == 0 && command[9] != '\0') {
                pitchPID.Kp = atof(command + 9);
                client.println("OK");
              } else if (strncmp(command, "setPitchI", 9) == 0 && command[9] != '\0') {
                pitchPID.Ki = atof(command + 9);
                client.println("OK");
              } else if (strncmp(command, "setPitchD", 9) == 0 && command[9] != '\0') {
                pitchPID.Kd = atof(command + 9);
                client.println("OK");
              } else if (strncmp(command, "setRollP", 8) == 0 && command[8] != '\0') {
                rollPID.Kp = atof(command + 8);
                client.println("OK");
              } else if (strncmp(command, "setRollI", 8) == 0 && command[8] != '\0') {
                rollPID.Ki = atof(command + 8);
                client.println("OK");
              } else if (strncmp(command, "setRollD", 8) == 0 && command[8] != '\0') {
                rollPID.Kd = atof(command + 8);
                client.println("OK");
              } else if (strcmp(command, "save") == 0) {
                saveParameters(); 
                client.println("OK");
              } else if (strcmp(command, "startPWMStream") == 0) {
                client.println("PWM_STREAM_START");
                pwmStreaming = true;
                cubeStreaming = false; 
              } else if (strcmp(command, "stopPWMStream") == 0) {
                pwmStreaming = false;
                client.println("PWM_STREAM_STOPPED");
              }else if (strcmp(command, "startCubeStream") == 0) {
                pwmStreaming = false;
                client.println("CUBE_STREAM_START");
                cubeStreaming = true;
              } else if (strcmp(command, "stopCubeStream") == 0) {
                client.println("CUBE_STREAM_STOPPED");
                cubeStreaming = false;
              } else {
                client.println("ERR");
              }
              
              xSemaphoreGive(configMutex);
              
          } else {
              Serial.println("Warning: Config Mutex unavailable.");
              client.println("ERR: Config access blocked.");
          }
        }
        
        // PWM streaming logic
        if (pwmStreaming) {
          uint32_t pulseWidth1 = pulseIn(PITCH_IP, HIGH, PULSE_TIMEOUT);
          uint32_t pulseWidth2 = pulseIn(ROLL_IP, HIGH, PULSE_TIMEOUT);
          uint32_t pulseWidth3 = pulseIn(YAW_IP, HIGH, PULSE_TIMEOUT);
          uint32_t pulseWidth4 = pulseIn(AUTO_PILOT, HIGH, PULSE_TIMEOUT);

          int p1 = constrain(map(pulseWidth1, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
          int p2 = constrain(map(pulseWidth2, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
          int p3 = constrain(map(pulseWidth3, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);
          int p4 = constrain(map(pulseWidth4, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH, 0, 100), 0, 100);

          if (p1 != lastPercentage1 || p2 != lastPercentage2 || p3 != lastPercentage3 || p4 != lastPercentage4 || millis() - lastSent > 200) {
            client.printf("%d,%d,%d,%d\n", p1, p2, p3, p4);
            lastPercentage1 = p1;
            lastPercentage2 = p2;
            lastPercentage3 = p3;
            lastPercentage4 = p4;
            lastSent = millis();
          }

          if (millis() - lastSent > 2000) {
            client.println("No signal");
            lastSent = millis();
          }
        }

        // Cube (IMU) streaming logic
        if (cubeStreaming && (millis() - lastSent) >= (1000 / FREQ)) {
          // gx, gy, gz are guaranteed to be fresh because loop() (Core 1) calls updateMPU6050() unconditionally.
          client.printf("%.2f,%.2f,%.2f\n", gx, gy, gz);
          lastSent = millis(); 
        }
        
        vTaskDelay(pdMS_TO_TICKS(1)); 
      }

      client.stop();
      pwmStreaming = false;
      cubeStreaming = false;
      Serial.println("Client disconnected.");
    }
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}


//--------------------PID Control----------------------------
// Function to compute PID output with bumpless transfer
float computePID(PID& pid, float setpoint, float input, float currentOutput) {
  unsigned long now = millis();
  float dt = (now - pid.last_time) / 1000.0;
  if (dt <= 0) dt = 0.001;// prevent division by zero
  
  float error = setpoint - input;

  // Bumpless transfer: adjust integral to match current output
  // It recalculates the integral term so that the current PID output matches the existing output (currentOutput).
  if (pid.Ki != 0) {
    pid.integral = (currentOutput - pid.Kp * error - pid.Kd * (error - pid.previous_error)/dt) / pid.Ki;
  }
  // Accumulates error over time. The integral term helps eliminate steady-state error.
  // constrained to prevent integral windup, which causes overshooting
  pid.integral += error * dt;
  pid.integral = constrain(pid.integral, -50, 50);// Anti-windup

  // Measures how fast the error is changing. Predicts the future behavior of the error.
  float derivative = (error - pid.previous_error) / dt;

  // Standard PID formula: P + I + D. Each component contributes to the final output value (e.g., servo angle).
  float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
  
  pid.previous_error = error;
  pid.last_time = now;
  
  return output;
}

// Reset PID controller
//to avoid unwanted behavior(overshoots because of accumulated error) when control modes change or conditions reset.
void resetPID(PID &pid) {
  pid.integral = 0;
  pid.previous_error = 0;
  pid.last_time = millis();
}

//--------------------Servo Control--------------------------
// NOTE: updateMPU6050() call has been removed from here, 
// as it is now handled by the main loop()
void updateServoFromMPU() {
  // gx and gy are fresh from loop() -> updateMPU6050()
  
  float pitchOutput = computePID(pitchPID, referencePitch, -gy, pitchServo.read() - pitchOffset);
  float rollOutput = computePID(rollPID, referenceRoll, -gx, rollServo.read() - rollOffset);

  int pitchAngle = constrain(pitchOffset + pitchOutput, 45, 135);
  int rollAngle = constrain(rollOffset + rollOutput, 45, 135);

  pitchServo.write(pitchAngle);
  rollServo.write(rollAngle);
}


// -------------------- MPU6050 Functions --------------------
void initMPU6050() {
  // Initializes the MPU6050 by configuring its internal registers using I²C communication.
  Serial.println("Initiating MPU6050 sensor...");
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x6b, 0x00); // Wakes up the MPU6050 by writing 0x00 to the power management register (0x6B). This is necessary because the MPU6050 starts in sleep mode by default to save power, and this command activates it for normal operation.
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1a, 0x03); // Configures the digital low-pass filter (DLPF) settings. Writing 0x03 to register 0x1A sets the DLPF to a bandwidth of 44 Hz for the gyroscope and 44 Hz for the accelerometer, which helps reduce noise in the sensor readings while still allowing for responsive measurements suitable for flight stabilization.
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1b, 0x08); // Sets the gyroscope sensitivity. Writing 0x08 to register 0x1B configures the gyroscope to a full-scale range of ±500 degrees per second (dps). This means that the raw gyroscope readings will be scaled such that the maximum measurable angular velocity corresponds to ±500 dps, providing a good balance between sensitivity and range for typical flight stabilization applications.
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x1c, 0x08); // Sets the accelerometer sensitivity. Writing 0x08 to register 0x1C configures the accelerometer to a full-scale range of ±4g. This means that the raw accelerometer readings will be scaled such that the maximum measurable acceleration corresponds to ±4 times the acceleration due to gravity (g), which is suitable for detecting typical movements and orientations in a flight stabilizer while providing enough resolution for accurate angle estimation.
  uint8_t sample_div = (1000 / FREQ) - 1;// Calculates the sample rate divider based on the desired output frequency (FREQ). The MPU6050's internal sample rate is 1 kHz, and the sample rate divider allows you to reduce this rate to match your control loop frequency. For example, if FREQ is set to 50 Hz, the sample_div will be calculated as (1000 / 50) - 1 = 19, which means the actual output rate from the MPU6050 will be 1000 / (19 + 1) = 50 Hz. This ensures that the sensor data is updated at a consistent rate that matches the control loop, providing timely and accurate readings for stabilization.
  i2c_write_reg(MPU6050_I2C_ADDRESS, 0x19, sample_div);// Writes the calculated sample rate divider to register 0x19 (SMPLRT_DIV) to set the output data rate of the MPU6050. This ensures that the sensor updates its readings at the desired frequency (FREQ), which is crucial for maintaining a responsive and stable control loop in the flight stabilizer. By adjusting this divider, you can optimize the balance between sensor update rate and processing load on the microcontroller.
}

void calibrateMPU6050() {
  // This function measures those biases and stores them so they can be subtracted from future readings.
  Serial.println("Calibrating MPU6050 Gyro...");
  int num = 500;
  long xSum = 0, ySum = 0, zSum = 0;
  uint8_t data[6];// Buffer to hold raw gyro data (6 bytes for X, Y, Z)

  for (int i = 0; i < num; i++) {
    if (i2c_read(MPU6050_I2C_ADDRESS, 0x43, data, 6) != 0) return;// Reads raw gyro data starting from register 0x43 (GYRO_XOUT_H) into the data buffer. This includes 6 bytes: high and low for X, Y, and Z axes. If the read operation fails, the function returns early, which means calibration will not be completed and offsets will remain at their initial value of 0.
    xSum += ((data[0] << 8) | data[1]);// Constructs 16-bit(2 byte) signed integers from the high and low bytes for each axis and accumulates them into xSum, ySum, and zSum. This process is repeated for 500 samples to get an average value for each axis, which represents the bias (offset) of the gyroscope when it is stationary.
    ySum += ((data[2] << 8) | data[3]);
    zSum += ((data[4] << 8) | data[5]);
    delay(2);
  }
  gyrXoffs = xSum / num; // Calculates the average offset for each axis by dividing the accumulated sums by the number of samples. These offsets represent the bias in the gyroscope readings when the device is stationary and are stored in gyrXoffs, gyrYoffs, and gyrZoffs. Future gyroscope readings will have these offsets subtracted to provide more accurate measurements of angular velocity.
  gyrYoffs = ySum / num;
  gyrZoffs = zSum / num;
  Serial.printf("Calibration finished. Offsets: X=%.2f, Y=%.2f, Z=%.2f\n", gyrXoffs, gyrYoffs, gyrZoffs);
}

void updateMPU6050() {
  //Reads raw sensor data(6 axis), filters it and computes orientation angles and updates gx,gy,gz
  static unsigned long last_time = millis(); //remembers when the function last ran(time stamp)
  uint8_t data[14];// Buffer to hold raw accelerometer, temperature, and gyroscope data (14 bytes total: 6 for accel, 2 for temp, 6 for gyro)

  if (i2c_read(MPU6050_I2C_ADDRESS, 0x3b, data, 14) != 0) return;//reads 14 byts(acc6,temp2,gyro6)

  accX = ((data[0] << 8) | data[1]); //constructiong 16 bit int from high and low byts, this is raw acc readings in x,y,z
  accY = ((data[2] << 8) | data[3]); 
  accZ = ((data[4] << 8) | data[5]);

  gyrX = (((data[8] << 8) | data[9]) - gyrXoffs) / gSensitivity; //Similar to above, Offsets(gyrXoffs, etc.) are subtracted(rm biases),continued below...
  gyrY = (((data[10] << 8) | data[11]) - gyrYoffs) / gSensitivity; //gSensitivity = 65.5, converting raw data to deg/s assuming +/-500 dps sensitivity.
  gyrZ = (((data[12] << 8) | data[13]) - gyrZoffs) / gSensitivity;

  //Low-Pass Filter Accelerometer Data
  filtered_ax = filtered_ax * (1.0 - ACCEL_FILTER) + accX * ACCEL_FILTER;//ACCEL_FILTER (like 0.3) controls how much "new" data affects the value.
  filtered_ay = filtered_ay * (1.0 - ACCEL_FILTER) + accY * ACCEL_FILTER;
  filtered_az = filtered_az * (1.0 - ACCEL_FILTER) + accZ * ACCEL_FILTER;

  //Low-Pass Filter Gyroscope Data
  filtered_gx = filtered_gx * (1.0 - GYRO_FILTER) + gyrX * GYRO_FILTER;//same as above, removes spikes/noise from the gyroscope data.
  filtered_gy = filtered_gy * (1.0 - GYRO_FILTER) + gyrY * GYRO_FILTER;
  filtered_gz = filtered_gz * (1.0 - GYRO_FILTER) + gyrZ * GYRO_FILTER;

  //Estimate Pitch and Roll from Accelerometer (Angle from Gravity), calculating tilt angle in degree
  double ay = atan2(filtered_ax, sqrt(pow(filtered_ay, 2) + pow(filtered_az, 2))) * 180 / M_PI;
  double ax = atan2(filtered_ay, sqrt(pow(filtered_ax, 2) + pow(filtered_az, 2))) * 180 / M_PI;

  //Integrate Gyroscope Readings to Track Rotation Over Time
  gx += filtered_gx / FREQ;//Integrates angular velocity (°/s) over time → gives angle in degrees.
  gy -= filtered_gy / FREQ;//Using FREQ to convert to time step: 1 / FREQ = seconds per update.
  gz += filtered_gz / FREQ;

  //Complementary Filter to Combine Gyro and Accelerometer
  gx = gx * (1.0 - COMP_FILTER) + ax * COMP_FILTER;
  gy = gy * (1.0 - COMP_FILTER) + ay * COMP_FILTER;

  //Ensures the function takes exactly 1 / FREQ seconds (e.g., 20 ms for 50 Hz). Ensures consistent sampling for filters and integration
  while (millis() - last_time < (1000 / FREQ)) delay(1);
  last_time = millis();
}

// -------------------- EEPROM --------------------
void loadParameters() {
  //loads value from EEPROM and also checks if the value loaded is valid
  if (xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {// Takes the mutex before accessing EEPROM to ensure thread safety between Core 0 and Core 1 when reading configuration parameters. This prevents data corruption and ensures that the values read from EEPROM are consistent and not being modified by another task at the same time.
      EEPROM.get(ACCEL_FILTER_ADDR, ACCEL_FILTER); // Reads the saved accelerometer filter coefficient from EEPROM and stores it in the ACCEL_FILTER variable. This value is used in the low-pass filter applied to the accelerometer data to smooth out noise. The function also checks if the loaded value is valid (not NaN and within a reasonable range) and resets it to a default if it's not, ensuring stable operation of the flight stabilizer.
      EEPROM.get(GYRO_FILTER_ADDR, GYRO_FILTER);// Similar to above, this reads the gyroscope filter coefficient from EEPROM and validates it. The GYRO_FILTER value is crucial for smoothing the gyroscope data, which can be noisy, especially in flight conditions. By allowing this value to be saved and loaded, users can tune the responsiveness of the gyroscope filtering to their specific needs and preferences.
      EEPROM.get(COMP_FILTER_ADDR, COMP_FILTER);// This reads the complementary filter coefficient from EEPROM and validates it. The COMP_FILTER value determines how much weight is given to the accelerometer data versus the gyroscope data when calculating the orientation angles (pitch and roll). By saving this value in EEPROM, users can adjust the balance between gyro and accelerometer data for optimal performance based on their specific use case and flying style.
      EEPROM.get(PITCH_PID_ADDR, pitchPID.Kp);
      EEPROM.get(PITCH_PID_ADDR + 4, pitchPID.Ki);// These lines read the PID gains for the pitch controller from EEPROM and store them in the pitchPID structure. The Kp, Ki, and Kd values are essential for tuning the PID controller's response to errors in pitch angle. By allowing these values to be saved and loaded from EEPROM, users can fine-tune the pitch control performance of their flight stabilizer and have those settings persist across reboots.
      EEPROM.get(PITCH_PID_ADDR + 8, pitchPID.Kd);
      EEPROM.get(ROLL_PID_ADDR, rollPID.Kp);
      EEPROM.get(ROLL_PID_ADDR + 4, rollPID.Ki);
      EEPROM.get(ROLL_PID_ADDR + 8, rollPID.Kd);

      // Validate loaded values
      if (isnan(ACCEL_FILTER) || ACCEL_FILTER <= 0 || ACCEL_FILTER > 1.0) ACCEL_FILTER = 0.3;
      if (isnan(GYRO_FILTER) || GYRO_FILTER <= 0 || GYRO_FILTER > 1.0) GYRO_FILTER = 0.08;
      if (isnan(COMP_FILTER) || COMP_FILTER <= 0 || COMP_FILTER > 1.0) COMP_FILTER = 0.7;
      
      if (isnan(pitchPID.Kp) || pitchPID.Kp < 0) pitchPID.Kp = 2.0;
      if (isnan(pitchPID.Ki) || pitchPID.Ki < 0) pitchPID.Ki = 0.5;
      if (isnan(pitchPID.Kd) || pitchPID.Kd < 0) pitchPID.Kd = 0.5;
      
      if (isnan(rollPID.Kp) || rollPID.Kp < 0) rollPID.Kp = 2.0;
      if (isnan(rollPID.Ki) || rollPID.Ki < 0) rollPID.Ki = 0.1;
      if (isnan(rollPID.Kd) || rollPID.Kd < 0) rollPID.Kd = 0.5;
      
      xSemaphoreGive(configMutex);// Releases the mutex after loading and validating the parameters, allowing other tasks (like the main control loop) to access these configuration values safely. This ensures that there are no conflicts or data corruption when multiple tasks need to read or write these shared parameters.
  }
}

void saveParameters() {
  if (xSemaphoreTake(configMutex, portMAX_DELAY) == pdTRUE) {
    EEPROM.put(ACCEL_FILTER_ADDR, ACCEL_FILTER);
    EEPROM.put(GYRO_FILTER_ADDR, GYRO_FILTER);
    EEPROM.put(COMP_FILTER_ADDR, COMP_FILTER);
    EEPROM.put(PITCH_PID_ADDR, pitchPID.Kp);
    EEPROM.put(PITCH_PID_ADDR + 4, pitchPID.Ki);
    EEPROM.put(PITCH_PID_ADDR + 8, pitchPID.Kd);
    EEPROM.put(ROLL_PID_ADDR, rollPID.Kp);
    EEPROM.put(ROLL_PID_ADDR + 4, rollPID.Ki);
    EEPROM.put(ROLL_PID_ADDR + 8, rollPID.Kd);
    EEPROM.commit();
    Serial.println("Parameters saved to EEPROM.");
    xSemaphoreGive(configMutex);
  }
}

// -------------------- I2C Helpers --------------------
int i2c_read(int addr, int start, uint8_t* buffer, int size) {
  Wire.beginTransmission(addr);
  Wire.write(start);
  if (Wire.endTransmission(false) != 0) return -1;
  //  Wire.requestFrom(addr, size, true); 
  Wire.requestFrom((uint8_t)addr, (size_t)size, (bool)true);
  int i = 0;
  while (Wire.available() && i < size) buffer[i++] = Wire.read();
  return (i == size) ? 0 : -1;
}

int i2c_write_reg(int addr, int reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  return Wire.endTransmission(true);
}