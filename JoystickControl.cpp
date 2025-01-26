#include <Dynamixel2Arduino.h>   // Library for controlling Dynamixel motors
#include <tuple>                 // For std::tuple usage
using std::tuple;
using std::make_tuple;
using std::get;

#if defined(ARDUINO_OpenRB)  // If using the OpenRB-150 board
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#endif

//----------------------------------
// Function Prototypes
//----------------------------------
tuple<int, int> homeMotor(int DXL_ID);
bool detectNoMovementWithinTurns(int DXL_ID, int delay_time, int no_movement_threshold, int movement_threshold, int max_position_units);
void calculateErrors(); // (Currently unused - for future error calculations)
int scaleToMotorRange(double angle, int cw_limit, int ccw_limit);
void initializeThetas();

//----------------------------------
// Joystick and Switch Configuration
//----------------------------------
const int JOYSTICK_X_PIN = A0;    // X-axis analog pin
const int JOYSTICK_Y_PIN = A1;    // Y-axis analog pin
const int JOYSTICK_SW_PIN = A2;   // Joystick switch pin
const int ENABLE_SWITCH_PIN = A3; // Additional switch pin

// Joystick center values and dead zones
const int JOYSTICK_CENTER_X = 732;
const int JOYSTICK_CENTER_Y = 756;
const int JOYSTICK_CENTER_RANGE_X_MIN = 731;
const int JOYSTICK_CENTER_RANGE_X_MAX = 733;
const int JOYSTICK_CENTER_RANGE_Y_MIN = 755;
const int JOYSTICK_CENTER_RANGE_Y_MAX = 757;
const int JOYSTICK_DEADZONE = 100;

// Control angles and increments
const float MAX_ANGLE_INCREMENT_ROLL_PITCH = 10.0; // deg/update
const float MAX_ANGLE_INCREMENT_JAWS = 10.0;       // deg/update
bool jawControlMode = false;                       // Toggles between roll/pitch mode and jaw mode
bool previousSWState = HIGH;                       // Tracks previous switch state

//----------------------------------
// Dynamixel Configuration
//----------------------------------
const uint8_t DXL_IDs[] = {1, 2, 3, 4};            // Motor IDs
const int NUM_MOTORS = sizeof(DXL_IDs) / sizeof(DXL_IDs[0]);
const float DXL_PROTOCOL_VERSION = 2.0;
const int DXL_BAUD_RATE = 57600;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//----------------------------------
// Motion and Safety Limits
//----------------------------------
using namespace ControlTableItem;
const int MAX_CURRENT = 170;  // Safe current limit for motors (in milliamps)
const float MAX_POSITION_RANGE = 600; // Max range in degrees (for safety)
const bool POSITIVE_IS_CCW = true;    // Rotation sign convention

// Arrays to store motor CW/CCW limits (updated after homing)
int cw_limits[NUM_MOTORS];
int ccw_limits[NUM_MOTORS];

//----------------------------------
// Transmission Matrix
//----------------------------------
// Maps user input (theta_R, theta_W, etc.) to motor angles
double transmissionMatrix[4][4] = {
  {0,    0,    0,    1.54},
  {0,    0,    1.03, 0   },
  {0,    1.15, -0.743,0  },
  {1.15, 0,    -0.743,0  }
};

// Inverse of the above matrix to compute motor angles from desired angles
double inverseMatrix[4][4] = {
  {0,               0.6272688898269312, 0,               0.8695652173913043},
  {0,               0.6272688898269312, 0.8695652173913043, 0             },
  {0,               0.970873786407767,  0,               0               },
  {0.6493506493506493, 0,               0,               0               }
};

//----------------------------------
// Angle Variables
//----------------------------------
double theta_R;  // Roll
double theta_W;  // Wrist (pitch)
double theta_G1; // Gripper half 1
double theta_G2; // Gripper half 2

//----------------------------------
// Loop Timing
//----------------------------------
unsigned long lastControlTime = 0;
const unsigned long controlInterval = 40; // in milliseconds

//----------------------------------
// Arduino Setup
//----------------------------------
void setup() {
  delay(1000);
  Serial.begin(57600);

  // Configure joystick and switches
  pinMode(JOYSTICK_SW_PIN, INPUT_PULLUP);
  pinMode(ENABLE_SWITCH_PIN, INPUT_PULLUP);
  previousSWState = digitalRead(JOYSTICK_SW_PIN);

  // Initialize Dynamixel
  dxl.begin(DXL_BAUD_RATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Homing sequence for each motor
  for (int i = 0; i < NUM_MOTORS; i++) {
    int DXL_ID = DXL_IDs[i];
    if (dxl.ping(DXL_ID)) {
      Serial.print("Dynamixel with ID: ");
      Serial.print(DXL_ID);
      Serial.println(" is connected!");

      // Set motor to Current Control Mode for homing
      dxl.torqueOff(DXL_ID);
      dxl.setOperatingMode(DXL_ID, OP_CURRENT);
      dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, MAX_CURRENT);
      dxl.writeControlTableItem(HOMING_OFFSET, DXL_ID, 0);

      // Home this motor
      tuple<int, int> limits = homeMotor(DXL_ID);
      cw_limits[i] = get<0>(limits);
      ccw_limits[i] = get<1>(limits);
    } 
    else {
      Serial.print("Failed to connect to Dynamixel with ID: ");
      Serial.println(DXL_ID);
    }
  }

  // After homing, set motors to Current-Based Position Mode
  for (int i = 0; i < NUM_MOTORS; i++) {
    int DXL_ID = DXL_IDs[i];
    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_CURRENT_BASED_POSITION);
    dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID, MAX_CURRENT);
    dxl.torqueOn(DXL_ID);
  }

  // Initialize thetas based on motor positions
  initializeThetas();
}

//----------------------------------
// Main Control Loop
//----------------------------------
void loop() {
  // Ensure the loop runs at a fixed interval
  if (millis() - lastControlTime >= controlInterval) {
    lastControlTime = millis();

    // Detect joystick switch press to toggle jaw mode
    bool currentSWState = digitalRead(JOYSTICK_SW_PIN);
    if (previousSWState == HIGH && currentSWState == LOW) {
      jawControlMode = !jawControlMode;
      Serial.print("Jaw control mode: ");
      Serial.println(jawControlMode ? "ON" : "OFF");
    }
    previousSWState = currentSWState;

    // Read enable switch and joystick
    bool enableSwitchState = digitalRead(ENABLE_SWITCH_PIN);
    int joystickX = analogRead(JOYSTICK_X_PIN);
    int joystickY = analogRead(JOYSTICK_Y_PIN);

    // Check if joystick is near center
    bool isJoystickCentered = 
        (joystickX >= JOYSTICK_CENTER_RANGE_X_MIN && joystickX <= JOYSTICK_CENTER_RANGE_X_MAX) &&
        (joystickY >= JOYSTICK_CENTER_RANGE_Y_MIN && joystickY <= JOYSTICK_CENTER_RANGE_Y_MAX);

    // Only process joystick if switch is not pressed and joystick is not centered
    if (enableSwitchState == HIGH && !isJoystickCentered) {
      int xDeviation = joystickX - JOYSTICK_CENTER_X;
      int yDeviation = joystickY - JOYSTICK_CENTER_Y;

      // Apply deadzone
      if (abs(xDeviation) < JOYSTICK_DEADZONE) xDeviation = 0;
      if (abs(yDeviation) < JOYSTICK_DEADZONE) yDeviation = 0;

      // Update angles if there's significant deviation
      if (xDeviation != 0 || yDeviation != 0) {
        const float MAX_JOYSTICK_DEVIATION = 512 - JOYSTICK_DEADZONE;
        float xAngleIncrement, yAngleIncrement;

        // If jaws are controlled, only horizontal is used by default
        if (jawControlMode) {
          xAngleIncrement = ((float)xDeviation / MAX_JOYSTICK_DEVIATION) * MAX_ANGLE_INCREMENT_JAWS;
          yAngleIncrement = 0; // No vertical by default in jaws mode
        } else {
          // Roll/Pitch control
          xAngleIncrement = ((float)xDeviation / MAX_JOYSTICK_DEVIATION) * MAX_ANGLE_INCREMENT_ROLL_PITCH;
          yAngleIncrement = ((float)yDeviation / MAX_JOYSTICK_DEVIATION) * MAX_ANGLE_INCREMENT_ROLL_PITCH;
        }

        // Update angles based on current mode
        if (jawControlMode) {
          // Jaws: open/close with horizontal movement
          theta_G1 -= xAngleIncrement; 
          theta_G2 += xAngleIncrement;

          // If vertical input is desired for jaws, shift both jaws
          if (yDeviation != 0) {
            float yJawIncrement = ((float)yDeviation / MAX_JOYSTICK_DEVIATION) * MAX_ANGLE_INCREMENT_JAWS;
            double jawsMid = (theta_G1 + theta_G2) / 2.0;
            double jawsHalfGap = (theta_G2 - theta_G1) / 2.0;
            jawsMid += yJawIncrement;
            theta_G1 = jawsMid - jawsHalfGap;
            theta_G2 = jawsMid + jawsHalfGap;
          }

        } else {
          // Roll (theta_R) and Pitch (theta_W)
          theta_R += xAngleIncrement;
          theta_W += yAngleIncrement;
        }

        // Enforce a minimum gap between G1 and G2
        if (theta_G2 > theta_G1) {
          float midAngle = (theta_G1 + theta_G2) / 2;
          theta_G1 = midAngle - 15; 
          theta_G2 = midAngle + 15;
        }

        // Constrain angles to avoid physical damage
        theta_R = constrain(theta_R, -90, 90);
        theta_W = constrain(theta_W, -95, 95);
        theta_G1 = constrain(theta_G1, -200, 190);
        theta_G2 = constrain(theta_G2, -190, 200);
      }
    }

    // Convert degrees to radians
    double theta_R_rad = theta_R * PI / 180.0;
    double theta_W_rad = theta_W * PI / 180.0;
    double theta_G1_rad = theta_G1 * PI / 180.0;
    double theta_G2_rad = theta_G2 * PI / 180.0;

    // Desired joint angles in array form
    double desiredAngles[4] = {theta_R_rad, theta_W_rad, theta_G1_rad, theta_G2_rad};
    double motorAngles[4] = {0};

    // Compute motor angles using the inverse transmission matrix
    for (int i = 0; i < 4; i++) {
      motorAngles[i] = 0;
      for (int j = 0; j < 4; j++) {
        motorAngles[i] += inverseMatrix[i][j] * desiredAngles[j];
      }
    }

    // Convert computed motor angles back to degrees
    for (int i = 0; i < 4; i++) {
      motorAngles[i] *= 180.0 / PI;
    }

    // Scale angles to motor positions
    int motor_positions[NUM_MOTORS];
    motor_positions[0] = scaleToMotorRange(motorAngles[2], cw_limits[0], ccw_limits[0]); // Motor ID 1
    motor_positions[1] = scaleToMotorRange(motorAngles[1], cw_limits[1], ccw_limits[1]); // Motor ID 2
    motor_positions[2] = scaleToMotorRange(motorAngles[0], cw_limits[2], ccw_limits[2]); // Motor ID 3
    motor_positions[3] = scaleToMotorRange(motorAngles[3], cw_limits[3], ccw_limits[3]); // Motor ID 4

    // Send positions and current limit to each motor
    for (int i = 0; i < NUM_MOTORS; i++) {
      dxl.setGoalPosition(DXL_IDs[i], motor_positions[i]);
      dxl.setGoalCurrent(DXL_IDs[i], MAX_CURRENT);
    }
  }
}

//----------------------------------
// Utility Functions
//----------------------------------

/**
 * @brief Scales a given angle (degrees) to the motor's position range.
 */
int scaleToMotorRange(double angle, int cw_limit, int ccw_limit) {
  // Convert angle (deg) to raw position
  int position = angle * 4096 / 360.0;

  int effective_range = abs(ccw_limit - cw_limit);
  int center_position = (cw_limit + ccw_limit) / 2;
  int half_range = effective_range / 2;

  // Constrain within half of the effective range
  position = constrain(position, -half_range, half_range);

  // Adjust sign based on POSITIVE_IS_CCW
  int scaled_position;
  if (POSITIVE_IS_CCW) {
    scaled_position = center_position + position;
  } else {
    scaled_position = center_position - position;
  }
  return scaled_position;
}

/**
 * @brief Performs homing for one motor by running it to CW and CCW stops.
 * @return A tuple (correctedCWLimit, correctedCCWLimit).
 */
tuple<int, int> homeMotor(int DXL_ID) {
  const int max_turns = 3 * 4096;
  const int max_current = MAX_CURRENT;
  const int no_movement_threshold = 10;
  const int delay_time = 50;
  const int movement_threshold = 2;
  const int max_position_units = (MAX_POSITION_RANGE * 4096) / 360.0;

  int cw_limit = -1;
  int ccw_limit = -1;

  Serial.print("Starting homing for motor with ID: ");
  Serial.println(DXL_ID);

  // Disable torque for other motors
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (DXL_IDs[i] != DXL_ID) {
      dxl.torqueOff(DXL_IDs[i]);
    }
  }

  // Enable torque for the current motor
  dxl.torqueOn(DXL_ID);

  // Move in CW direction to find limit
  Serial.println("Homing in CW direction...");
  dxl.setGoalCurrent(DXL_ID, -max_current, UNIT_MILLI_AMPERE);
  if (detectNoMovementWithinTurns(DXL_ID, delay_time, no_movement_threshold, movement_threshold, max_turns)) {
    cw_limit = dxl.getPresentPosition(DXL_ID);
    Serial.print("CW Limit found: ");
    Serial.println(cw_limit);
  } else {
    Serial.println("Failed to find CW limit.");
    return make_tuple(-1, -1);
  }

  // Move in CCW direction to find limit
  Serial.println("Homing in CCW direction...");
  dxl.setGoalCurrent(DXL_ID, max_current, UNIT_MILLI_AMPERE);
  if (detectNoMovementWithinTurns(DXL_ID, delay_time, no_movement_threshold, movement_threshold, max_position_units)) {
    ccw_limit = dxl.getPresentPosition(DXL_ID);
    Serial.print("CCW Limit found: ");
    Serial.println(ccw_limit);
  } else {
    Serial.println("Failed to find CCW limit.");
    return make_tuple(-1, -1);
  }

  // Calculate and apply homing offset
  int homing_offset = - (cw_limit + ccw_limit) / 2;
  dxl.torqueOff(DXL_ID);
  dxl.writeControlTableItem(HOMING_OFFSET, DXL_ID, homing_offset);
  dxl.writeControlTableItem(MIN_POSITION_LIMIT, DXL_ID, cw_limit + homing_offset);
  dxl.writeControlTableItem(MAX_POSITION_LIMIT, DXL_ID, ccw_limit + homing_offset);
  dxl.torqueOn(DXL_ID);

  Serial.print("Homing offset applied: ");
  Serial.println(homing_offset);
  Serial.print("Corrected CW Limit: ");
  Serial.println(cw_limit + homing_offset);
  Serial.print("Corrected CCW Limit: ");
  Serial.println(ccw_limit + homing_offset);

  return make_tuple(cw_limit + homing_offset, ccw_limit + homing_offset);
}

/**
 * @brief Detects motor stall (no movement) within a certain range and time.
 * @return true if no movement is detected, false if maximum turns are reached.
 */
bool detectNoMovementWithinTurns(int DXL_ID, int delay_time, int no_movement_threshold,
                                 int movement_threshold, int max_position_units) {
  int no_movement_counter = 0;
  int last_position = dxl.getPresentPosition(DXL_ID);
  int initial_position = last_position;

  while (true) {
    delay(delay_time);
    int current_position = dxl.getPresentPosition(DXL_ID);

    // Check if movement is below threshold
    if (abs(current_position - last_position) < movement_threshold) {
      no_movement_counter++;
      if (no_movement_counter >= no_movement_threshold) {
        return true;  // Limit found
      }
    } else {
      no_movement_counter = 0;  // Reset if movement resumes
    }

    // Stop if moved beyond max units
    if (abs(current_position - initial_position) >= max_position_units) {
      Serial.println("Maximum turn limit reached without finding a stop.");
      return false;
    }
    last_position = current_position;
  }
}

/**
 * @brief Reads current motor positions and initializes thetas for R, W, G1, G2.
 */
void initializeThetas() {
  double motorAngles[4] = {0};

  // Convert each motor's position to an angle
  for (int i = 0; i < NUM_MOTORS; i++) {
    int present_position = dxl.getPresentPosition(DXL_IDs[i]);
    int midpoint = (cw_limits[i] + ccw_limits[i]) / 2;
    int position = 0;

    // Adjust position based on sign convention
    if (POSITIVE_IS_CCW) {
      position = present_position - midpoint;
    } else {
      position = midpoint - present_position;
    }

    motorAngles[i] = (double)position / 4096.0 * 360.0;
  }

  // Calculate initial thetas from motor angles using the (forward) transmission matrix
  theta_R  = transmissionMatrix[0][3] * motorAngles[3];
  theta_W  = transmissionMatrix[1][2] * motorAngles[0];
  theta_G1 = transmissionMatrix[2][1] * motorAngles[1] + transmissionMatrix[2][2] * motorAngles[0];
  theta_G2 = transmissionMatrix[3][0] * motorAngles[2] + transmissionMatrix[3][2] * motorAngles[0];

  // Print out initial angles
  Serial.println("Initial Thetas:");
  Serial.print("theta_R: "); Serial.println(theta_R);
  Serial.print("theta_W: "); Serial.println(theta_W);
  Serial.print("theta_G1: "); Serial.println(theta_G1);
  Serial.print("theta_G2: "); Serial.println(theta_G2);
}


