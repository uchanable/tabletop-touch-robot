#if defined(MODE_INTEGRATED_CONTROL)

#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL SerialUSB
#endif

// ==========================================
// Feedforward trajectory data
// ==========================================
struct TrajectoryPoint {
  float position_y;
  float motor_angle;
};

const int TRAJECTORY_POINTS_FORWARD = 3;
const TrajectoryPoint g_trajectory_forward[TRAJECTORY_POINTS_FORWARD] = {
  // Paste learned data (forward path)
  // Example
  // {0.0000,-20.50},
  // {0.0110,-14.50},
  // {0.0211,-11.50},
};

const int TRAJECTORY_POINTS_BACKWARD = 3; 
const TrajectoryPoint g_trajectory_backward[TRAJECTORY_POINTS_BACKWARD] = {
  // Paste learned data (return path)
  // Example
  // {0.0192,-11.50},
  // {0.0089,-14.50},
  // {-0.0006,-19.00},
};


// ==========================================
// Odometry sensor calibration variables
// Higher values result in shorter travel distance; lower values result in longer travel distance.
// Example: Changing 0.935 to 0.945 changes travel from 12.5cm to 12cm.
// Tip: If actual travel distance is longer than set distance, increase the value; otherwise decrease it.
// Calibrate the forward path first, then the return path.
// ==========================================
const float OTOS_FWD = 0.935; // Forward path
const float OTOS_BWD = 1.020; // Return path


// ==========================================
// Constants and parameters
// ==========================================
float integ_force_target = 40.0;// Target force [g]

// PID parameters for force control
float integ_force_kp = 0.3;
float integ_force_ki =0.03;
float integ_force_kd = 0.03;

// Force control module range of motion [deg]
float integ_force_min_angle = -90.0;
float integ_force_max_angle = 150.0;
// PID parameters for heading angle control
float integ_robot_kp = 2000.0;
float integ_robot_ki = 1150.0;
float integ_robot_kd = 250.0;

float integ_base_speed = 198;// Target speed [pulse] = (target speed 3.0 [cm/s] / (motor no-load RPM 114 [rpm] / 60 [s] * wheel diameter 2.6*pi [cm] / 1024 [pulse]))
const unsigned long integ_control_interval_ms = 20;// Speed control period [ms]

float integ_target_speed = 0.03;// Target speed 3.0 [m/s]

// PID parameters for speed control
float integ_speed_kp = 100.0;
float integ_speed_ki = 70.0;
float integ_speed_kd = 0.3;

float integ_max_speed_correction = 1023.0;// Maximum speed correction
bool integ_velocity_control_enabled = true;

const int INTEG_SPEED_STARTUP_CYCLES = 10;// Time to reach target speed 0.2[s] / control period 0.02[s]

// Initial variables for accurate OTOS sensing
float otos_angle_offset = 0.0;
float otos_y_offset = 0.0;
float last_corrected_angle = 0.0;
float last_corrected_x = 0.0;
float last_corrected_y = 0.0;
float temp_y_before_switch = 0.0;
float temp_angle_before_switch = 0.0;
bool integ_scalar_switch_started = false;

float integ_default_distance_m = 0.12; // Travel distance 12cm
int integ_default_repeat_count = 10;   // Round trip count: 10
const float INTEG_ARM_LIFT_ANGLE = 90.0;// Initial standby angle for stroking module (to make it easier to insert the arm)

bool integ_force_control_active = false;
float integ_current_force = 0.0;
float integ_current_angle_id1 = 0.0;
float integ_target_angle_id1 = 0.0;
float integ_force_previous_error = 0.0;
float integ_force_integral_error = 0.0;
unsigned long integ_force_last_control_time = 0;
const int INTEG_FORCE_SAMPLES = 5;
float integ_force_buffer[INTEG_FORCE_SAMPLES];
int integ_force_buffer_index = 0;
unsigned long integ_last_force_read = 0;
unsigned long integ_last_force_control_update = 0;
const unsigned long INTEG_FORCE_READ_INTERVAL = 20;// Load cell reading interval [ms]
const unsigned long INTEG_FORCE_CONTROL_UPDATE_INTERVAL = 50;// Force control period [ms]
bool integ_log_data = false;
unsigned long integ_last_data_log_time = 0;
const unsigned long INTEG_DATA_LOG_INTERVAL = 100;// Data output interval [ms]
IntegPIDController integ_angle_pid(integ_robot_kp, integ_robot_ki, integ_robot_kd, 0.0);
sfe_otos_pose2d_t integ_current_position;
float integ_base_angle = 0.0;
bool integ_motors_initialized = false;
bool integ_sensor_initialized = false;
bool integ_robot_is_running = false;
bool integ_stop_flag = false;

// Robot state machine
enum RobotState {
  ROBOT_IDLE,
  ROBOT_FORWARD,
  ROBOT_FORWARD_WAIT,
  ROBOT_BACKWARD,
  ROBOT_BACKWARD_WAIT,
  ROBOT_CYCLE_COMPLETE
};
RobotState integ_robot_state = ROBOT_IDLE;

float integ_target_distance = 0.0;
int integ_repeat_count = 0;
int integ_current_cycle = 0;
unsigned long integ_last_robot_update = 0;
bool integ_system_initialized = false;
bool integ_hx711_available = false;
sfe_otos_pose2d_t integ_current_velocity;
float integ_speed_integral = 0.0;
float integ_speed_previous_error = 0.0;
unsigned long integ_last_velocity_control_time = 0;
float integ_actual_speed = 0.0;
float integ_speed_error = 0.0;
int integ_speed_startup_counter = 0;


// ==========================================
// Forward declarations
// ==========================================
void integUpdateRobotControl();
void integResetSpeedControl();
float integCalculateSpeedCorrection(float dt);
float integGetStartupSpeedScale();
void otosUpdateCorrectedPosition();
void stopMovementMotorsReliable();
void otosHandleForwardToBackward_Start();
void otosHandleForwardToBackward_Finish();
void otosHandleBackwardToForward_Start();
void otosHandleBackwardToForward_Finish();


// ==========================================
// Accurate OTOS sensing functions
// ==========================================
void otosUpdateCorrectedPosition() {
  myOtos.getPosition(integ_current_position);
  last_corrected_angle = integ_current_position.h;
  last_corrected_x = integ_current_position.x;
  // Add offset to maintain continuity
  last_corrected_y = integ_current_position.y;
}

// ---------------------------------------------------------
// When switching from forward to backward
// ---------------------------------------------------------
void otosHandleForwardToBackward_Start() {
  DEBUG_SERIAL.println("[OTOS] Forward->Backward: Starting scale change");
  otosUpdateCorrectedPosition();
  // 1. Change scalar (using setLinearScalar directly)
  myOtos.setLinearScalar(OTOS_BWD);
}

void otosHandleForwardToBackward_Finish() {
  // 2. Get raw coordinates after change
  myOtos.getPosition(integ_current_position);
  float y_after_raw = integ_current_position.y;
  float h_after_raw = integ_current_position.h;
  
  // 3. Calculate offset to correct drift
  // By using the saved "temp_y_before_switch" at stop,
  // cancel drift during wait and pull back to stop position
  otos_y_offset = temp_y_before_switch - y_after_raw;

  // 4. PID reset
  integ_angle_pid.reset();
  integ_angle_pid.setSetpoint(integ_base_angle);
  otosUpdateCorrectedPosition();
  DEBUG_SERIAL.print("Offset Updated: ");
  DEBUG_SERIAL.println(otos_y_offset, 5);
}

// ---------------------------------------------------------
// When switching from backward to forward
// ---------------------------------------------------------
void otosHandleBackwardToForward_Start() {
  DEBUG_SERIAL.println("[OTOS] Backward->Forward: Starting calibration");
  otosUpdateCorrectedPosition();
  // 1. Change scalar
  myOtos.setLinearScalar(OTOS_FWD);
}

void otosHandleBackwardToForward_Finish() {
  // 2. Get raw coordinates after change
  myOtos.getPosition(integ_current_position);
  float y_after_raw = integ_current_position.y;
  float h_after_raw = integ_current_position.h;
  
  // 3. Update offset
  otos_y_offset = temp_y_before_switch - y_after_raw;

  // 4. PID reset
  integ_angle_pid.reset();
  integ_angle_pid.setSetpoint(integ_base_angle);
  otosUpdateCorrectedPosition();
  DEBUG_SERIAL.print("[OTOS] Current Y: ");
  DEBUG_SERIAL.println(last_corrected_y, 4);
}

// ==========================================
// Reliable motor stop process (single send sometimes fails due to communication issues)
// ==========================================
void stopMovementMotorsReliable() {
  // Send 3 times consecutively
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
}


// ==========================================
// Feedforward trajectory functions
// ==========================================
// Forward path: Y coordinate increases from 0.0 to 0.12
float getFeedforwardAngle_Forward(float current_y) { 
  if (TRAJECTORY_POINTS_FORWARD <= 1) return g_trajectory_forward[0].motor_angle;

  // If below start point (0.0), return first angle
  if (current_y <= g_trajectory_forward[0].position_y) {
    return g_trajectory_forward[0].motor_angle;
  }
  
  // If above goal point (0.12), return last angle
  float last_y = g_trajectory_forward[TRAJECTORY_POINTS_FORWARD - 1].position_y;
  if (current_y >= last_y) {
    return g_trajectory_forward[TRAJECTORY_POINTS_FORWARD - 1].motor_angle;
  }
  
  // Interpolation in increasing range (y0 <= current <= y1)
  for (int i = 0; i < TRAJECTORY_POINTS_FORWARD - 1; i++) {
    float y0 = g_trajectory_forward[i].position_y;
    float y1 = g_trajectory_forward[i+1].position_y;
    
    // Since Y increases, y0 <= y1
    if (current_y >= y0 && current_y <= y1) {
      float a0 = g_trajectory_forward[i].motor_angle;
      float a1 = g_trajectory_forward[i+1].motor_angle;      
      if (abs(y1 - y0) < 0.0001) return a0;

      // Linear interpolation
      return a0 + (current_y - y0) * (a1 - a0) / (y1 - y0);
    }
  }
  return g_trajectory_forward[TRAJECTORY_POINTS_FORWARD - 1].motor_angle;
}

// Return path: Y coordinate decreases from 0.12 to 0.0
float getFeedforwardAngle_Backward(float current_y) {
  if (TRAJECTORY_POINTS_BACKWARD <= 1) return g_trajectory_backward[0].motor_angle;

  // If above start point (0.12), return first angle
  if (current_y >= g_trajectory_backward[0].position_y) {
    return g_trajectory_backward[0].motor_angle;
  }
  
  // If below goal point (0.0), return last angle
  float last_y = g_trajectory_backward[TRAJECTORY_POINTS_BACKWARD - 1].position_y;
  if (current_y <= last_y) {
    return g_trajectory_backward[TRAJECTORY_POINTS_BACKWARD - 1].motor_angle;
  }
  
  // Interpolation in decreasing range (y0 >= current >= y1)
  for (int i = 0; i < TRAJECTORY_POINTS_BACKWARD - 1; i++) {
    float y0 = g_trajectory_backward[i].position_y; // Larger value
    float y1 = g_trajectory_backward[i+1].position_y; // Smaller value
    
    if (current_y <= y0 && current_y >= y1) {
      float a0 = g_trajectory_backward[i].motor_angle;
      float a1 = g_trajectory_backward[i+1].motor_angle;      
      if (abs(y1 - y0) < 0.0001) return a0;

      // Linear interpolation
      return a0 + (current_y - y0) * (a1 - a0) / (y1 - y0);
    }
  }
  return g_trajectory_backward[TRAJECTORY_POINTS_BACKWARD - 1].motor_angle;
}


// ==========================================
// Setup
// ==========================================
void setupIntegratedControl() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL && millis() < 3000);
  
  DEBUG_SERIAL.println("\n=== MOBITOUCH Integrated Control System ===");
  DEBUG_SERIAL.println("Force control + mobile robot control");
  DEBUG_SERIAL.println("Version: v5.3 (Position Lock at Stop)");
  DEBUG_SERIAL.println("===================================\n");

  DEBUG_SERIAL.println("FF trajectory map loaded (Forward):");
  DEBUG_SERIAL.print("  ");
  DEBUG_SERIAL.print(TRAJECTORY_POINTS_FORWARD);
  DEBUG_SERIAL.println(" data points loaded.");
  
  DEBUG_SERIAL.println("FF trajectory map loaded (Backward):");
  DEBUG_SERIAL.print("  ");
  DEBUG_SERIAL.print(TRAJECTORY_POINTS_BACKWARD);
  DEBUG_SERIAL.println(" data points loaded.");
  
  DEBUG_SERIAL.println("HX711 initializing...");
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  delay(500);

  int retry_count = 0;
  bool hx711_ready = false;
  while (retry_count < 5 && !hx711_ready) {
    if (scale.is_ready()) {
      hx711_ready = true;
      break;
    }
    delay(500);
    retry_count++;
  }
  
  if (hx711_ready) {
    DEBUG_SERIAL.println("HX711 connection confirmed");
    scale.set_scale(-2538.48);// Load cell calibration value
    scale.tare();
    DEBUG_SERIAL.println("HX711 initialization complete");
    integ_hx711_available = true;
  } else {
    DEBUG_SERIAL.println("HX711 initialization failed");
    integ_hx711_available = false;
  }
  
  DEBUG_SERIAL.println("\nDynamixel motor initializing...");
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  delay(100);

  if (dxl.ping(MOTOR_ID)) {
    DEBUG_SERIAL.print("Force control motor (ID=");
    DEBUG_SERIAL.print(MOTOR_ID);
    DEBUG_SERIAL.println(") detected");
    dxl.torqueOff(MOTOR_ID);
    delay(100);
    dxl.setOperatingMode(MOTOR_ID, OP_POSITION);
    uint16_t cw_limit = 0;
    uint16_t ccw_limit = 1023;
    dxl.write(MOTOR_ID, 6, (uint8_t*)&cw_limit, 2);
    dxl.write(MOTOR_ID, 8, (uint8_t*)&ccw_limit, 2);
    dxl.torqueOn(MOTOR_ID);
    delay(100);
    uint16_t speed_value = 1023;
    dxl.write(MOTOR_ID, 32, (uint8_t*)&speed_value, 2);
  }
  
  if (dxl.ping(INTEG_LEFT_MOTOR_ID)) {
    dxl.torqueOff(INTEG_LEFT_MOTOR_ID);
    dxl.setOperatingMode(INTEG_LEFT_MOTOR_ID, OP_VELOCITY);
    dxl.torqueOn(INTEG_LEFT_MOTOR_ID);
  }
  
  if (dxl.ping(INTEG_RIGHT_MOTOR_ID)) {
    dxl.torqueOff(INTEG_RIGHT_MOTOR_ID);
    dxl.setOperatingMode(INTEG_RIGHT_MOTOR_ID, OP_VELOCITY);
    dxl.torqueOn(INTEG_RIGHT_MOTOR_ID);
  }
  
  integ_motors_initialized = true;
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
  
  DEBUG_SERIAL.println("\nOTOS sensor initializing...");
  Wire.begin();
  if (!myOtos.begin()) {
    DEBUG_SERIAL.println("Sensor connection failed");
    integ_sensor_initialized = false;
  } else {
    DEBUG_SERIAL.println("Sensor connection successful");
    myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
    myOtos.setAngularUnit(kSfeOtosAngularUnitRadians);

    sfe_otos_pose2d_t offset = {0, 0.02385, PI};// Odometry sensor position in robot frame {x[m], y[m], theta[rad]}
    myOtos.setOffset(offset);
    
    myOtos.setLinearScalar(OTOS_FWD);// Initial scalar setting (for forward)
    myOtos.setAngularScalar(0.995);
    
    myOtos.calibrateImu();
    delay(1000);
    myOtos.resetTracking();
    delay(100);
    
    // Initial offset reset
    otos_y_offset = 0.0;
    otos_angle_offset = 0.0;
    otosUpdateCorrectedPosition();
    
    // Set reference angle to corrected value
    integ_base_angle = last_corrected_angle;
    
    integ_sensor_initialized = true;
    DEBUG_SERIAL.println("Sensor initialization complete");
  }
  
  integ_current_angle_id1 = getCurrentAngle();
  integ_target_angle_id1 = integ_current_angle_id1;

  for (int i = 0; i < INTEG_FORCE_SAMPLES; i++) {
    integ_force_buffer[i] = 0.0;
  }
  
  integ_system_initialized = true;
  
  DEBUG_SERIAL.println("\n===================================");
  DEBUG_SERIAL.println("System initialization complete");
  DEBUG_SERIAL.println("===================================");
  
  DEBUG_SERIAL.println("\n[Force Control Settings]");
  DEBUG_SERIAL.print("  Target force: ");
  DEBUG_SERIAL.print(integ_force_target);
  DEBUG_SERIAL.println("g");
  
  DEBUG_SERIAL.println("\n[Mobile Robot Settings]");
  DEBUG_SERIAL.print("  Base speed: ");
  DEBUG_SERIAL.println(integ_base_speed);
  DEBUG_SERIAL.println("  Control: Absolute coordinate based");
  DEBUG_SERIAL.println("  Forward LinearScalar: 0.923");
  DEBUG_SERIAL.println("  Backward LinearScalar: 0.906");
  DEBUG_SERIAL.println("  AngularScalar: 0.991");
  
  DEBUG_SERIAL.println("\n[Speed Compensation Settings]");
  DEBUG_SERIAL.print("  Target speed: ");
  DEBUG_SERIAL.print(integ_target_speed * 100.0, 1);
  DEBUG_SERIAL.println(" cm/s");
  
  DEBUG_SERIAL.println("\n[Commands]");
  DEBUG_SERIAL.println("  c           - OTOS calibration");
  DEBUG_SERIAL.println("  force start - Start force control");
  DEBUG_SERIAL.println("  force stop  - Stop force control");
  DEBUG_SERIAL.println("  robot start - Start movement");
  DEBUG_SERIAL.println("  all start   - Start force control + movement simultaneously");
  DEBUG_SERIAL.println("  vel on/off  - Speed compensation ON/OFF");
  DEBUG_SERIAL.println("  Enter       - Emergency stop");
  DEBUG_SERIAL.println("");
}


// ==========================================
// Main loop
// ==========================================
// ---------------------------------------------------------
// Calibration process
// ---------------------------------------------------------
void integPerformCalibration() {
  DEBUG_SERIAL.println("[SEQ] Performing calibration...");
  if (integ_sensor_initialized) {
    myOtos.calibrateImu();
    delay(1000);
    myOtos.resetTracking();
    delay(500);
    
    // Offset reset
    otos_y_offset = 0.0;
    otos_angle_offset = 0.0;
    otosUpdateCorrectedPosition();    
    integ_base_angle = last_corrected_angle;
    otos_angle_offset = 0.0; // Adjust as needed
    DEBUG_SERIAL.println("Calibration complete");
  }
}

// ---------------------------------------------------------
// Function to slowly move stroking module to specified angle
// ---------------------------------------------------------
void integSlowApproachAngle(float target_angle, int delay_ms) {
  float current_angle = getCurrentAngle(); // Get current angle
  float step = 0.5; // Angle per step (smaller = smoother)  
  if (current_angle < target_angle) {
    for (float a = current_angle; a <= target_angle; a += step) {
      moveToAngle(a);
      delay(delay_ms);
    }
  } else {
    for (float a = current_angle; a >= target_angle; a -= step) {
      moveToAngle(a);
      delay(delay_ms);
    }
  }
  // Ensure final position at target value
  moveToAngle(target_angle);
}

void loopIntegratedControl() {
  if (!integ_system_initialized) return; 
  unsigned long current_time = millis();
  if (integ_sensor_initialized) {
    otosUpdateCorrectedPosition();
  }
  // Robot control (20ms interval)
  if (integ_robot_state != ROBOT_IDLE && integ_robot_state != ROBOT_CYCLE_COMPLETE) {
    bool is_waiting = (integ_robot_state == ROBOT_FORWARD_WAIT || integ_robot_state == ROBOT_BACKWARD_WAIT);    
    if (is_waiting || (current_time - integ_last_robot_update >= integ_control_interval_ms)) {
      if (!is_waiting) integ_last_robot_update = current_time;
      integUpdateRobotControl();
    }
  }
  
  if (DEBUG_SERIAL.available()) {
    String command = DEBUG_SERIAL.readStringUntil('\n');
    command.trim();

    if (command == "" || command.length() == 0) {
      DEBUG_SERIAL.println("\n!!! Emergency Stop !!!");
      integ_force_control_active = false;
      integ_stop_flag = true;
      stopMovementMotorsReliable();
      integ_robot_is_running = false;
      integ_log_data = false;
      integResetSpeedControl();
    } else {
      String cmd = command;
      cmd.toLowerCase();

      if (cmd == "c") {
        integPerformCalibration();
      }
      else if (cmd == "force start") {
        if (integ_hx711_available) {
          integ_force_control_active = true;
          integ_force_previous_error = 0;
          integ_force_integral_error = 0;
          integ_force_last_control_time = 0;          
          integ_log_data = true;
          DEBUG_SERIAL.println("[HEADER] time[ms],force[g],pos_y[m],pos_x[m],angle[deg],speed[cm/s]");          
          DEBUG_SERIAL.println("*** Force Control Start ***");
        }
      }
      else if (cmd == "force stop") {
        integ_force_control_active = false;
        
        integ_log_data = false;
        
        DEBUG_SERIAL.println("*** Force Control Stop ***");
      }
      else if (cmd == "all start" || cmd == "robot start") {
        
        DEBUG_SERIAL.println("\n=== Auto Start Sequence Begin ===");

        // 1. Initial calibration
        integPerformCalibration();

        // 2. Retract stroking module (raise)
        DEBUG_SERIAL.println("[SEQ] Raising arm...");
        integSlowApproachAngle(INTEG_ARM_LIFT_ANGLE, 5); 

        // 3. Wait for user input
        DEBUG_SERIAL.println("\n>>> Place the arm and press space + Enter <<<");
        while (!DEBUG_SERIAL.available());
        DEBUG_SERIAL.readStringUntil('\n');

        // Get angle from last point of return path data (near y=0)
        float start_angle = g_trajectory_forward[0].motor_angle;
        
        DEBUG_SERIAL.print("[SEQ] Descending to optimal angle: ");
        DEBUG_SERIAL.println(start_angle);
        integSlowApproachAngle(start_angle, 15);

        // 5. After descent, wait briefly and recalibrate
        delay(1000); 
        DEBUG_SERIAL.println("[SEQ] Final calibration...");
        integPerformCalibration();
        
        // =========================================================
        // Sensor value update + PID pre-convergence (warm-up)
        // =========================================================
        DEBUG_SERIAL.println("[SEQ] Force control warm-up start (10 seconds)...");
        if (integ_hx711_available) {
            // Buffer reset
            integ_force_buffer_index = 0;
            integ_force_previous_error = 0.0;
            integ_force_integral_error = 0.0;
            // --- Warm-up loop ---
            // Run force control only without moving robot, to accumulate integral error
            // =========================================================
            // Gain switching (stationary mode)
            // =========================================================
            float temp_ki_backup = integ_force_ki; // Save original setting
            integ_force_ki = 1.6;                  // Use strong integral to eliminate drift while stationary!
            // =========================================================

            unsigned long warmup_start = millis();
            float ff_angle_start = g_trajectory_forward[0].motor_angle;
            
            while (millis() - warmup_start < 5000) { // Run for 5 seconds
                // 1. Get force
                if (scale.is_ready()) {
                    float val = scale.get_units(1);
                    integ_force_buffer[integ_force_buffer_index] = val;
                    integ_force_buffer_index = (integ_force_buffer_index + 1) % INTEG_FORCE_SAMPLES;
                }
                
                // Averaging
                float sum = 0; 
                for(int i=0; i<INTEG_FORCE_SAMPLES; i++) sum += integ_force_buffer[i];
                integ_current_force = sum / INTEG_FORCE_SAMPLES;
                
                // 2. PID calculation (simplified)
                float error = integ_force_target - integ_current_force;
                // Accumulate I-term only (dt = control period 0.02[s])
                integ_force_integral_error += error * 0.02; 
                integ_force_integral_error = constrainFloat(integ_force_integral_error, -10, 10);
                
                // Output calculation (P + I)
                float pid_out = integ_force_kp * error + integ_force_ki * integ_force_integral_error;
                
                // Target angle = FF (forward initial value) - PID
                // This allows PID (especially I-term) to learn the deviation from FF
                float target = ff_angle_start - pid_out;
                target = constrainFloat(target, integ_force_min_angle, integ_force_max_angle);
                
                // Apply to motor
                moveToAngle(target);
                integ_target_angle_id1 = target; // Save to carry over state
                
                delay(20);
            }

            DEBUG_SERIAL.print("[SEQ] Warm-up complete. Force: ");
            DEBUG_SERIAL.print(integ_current_force);
            DEBUG_SERIAL.print("g, I-term: ");
            DEBUG_SERIAL.println(integ_force_integral_error);
        }
        // =========================================================

        DEBUG_SERIAL.println("[SEQ] Starting in 3 seconds...");
        delay(3000);// Set time before stroking starts [ms]
        // =========================================================
        // Gain switching (running mode)
        // =========================================================
        integ_force_ki = 0.03;       // Restore integral gain just before running
        integ_force_integral_error = 0.0;
        // =========================================================

        if (cmd == "all start") {
          if (integ_hx711_available) {
            integ_force_control_active = true;
            integ_force_previous_error = 0;
            integ_force_last_control_time = millis();
          }
          integ_log_data = true;
          DEBUG_SERIAL.println("[HEADER] time[ms],force[g],pos_y[m],pos_x[m],angle[deg],speed[cm/s]");
        }
        
        integ_target_distance = integ_default_distance_m; 
        integ_repeat_count = integ_default_repeat_count;
        
        integResetSpeedControl();
        integ_current_cycle = 0;
        integ_robot_is_running = true;
        integ_stop_flag = false;
        integ_robot_state = ROBOT_FORWARD;
        
        DEBUG_SERIAL.print("\n=== Mobile Robot Round Trip Start (Dist: ");
        DEBUG_SERIAL.print(integ_default_distance_m * 100.0);
        DEBUG_SERIAL.print("cm, Count: ");
        DEBUG_SERIAL.print(integ_repeat_count);
        DEBUG_SERIAL.println(") ===");
        
        // Initial scalar for forward
        myOtos.setLinearScalar(OTOS_FWD);
        
        otosUpdateCorrectedPosition();
        integ_angle_pid.reset();
        integ_angle_pid.setSetpoint(integ_base_angle);
        
        integ_scalar_switch_started = false;
      }
      else if (cmd == "vel on") {
        integ_velocity_control_enabled = true;
        DEBUG_SERIAL.println("*** Speed Compensation: Enabled ***");
      }
      else if (cmd == "vel off") {
        integ_velocity_control_enabled = false;
        integ_speed_integral = 0.0;
        integ_speed_previous_error = 0.0;
        DEBUG_SERIAL.println("*** Speed Compensation: Disabled ***");
      }
    }
  }
  
  // Force measurement
  if (current_time - integ_last_force_read >= INTEG_FORCE_READ_INTERVAL) {
    integ_last_force_read = current_time;
    if (integ_hx711_available && scale.is_ready()) {
      float new_force = scale.get_units(1);
      integ_force_buffer[integ_force_buffer_index] = new_force;
      integ_force_buffer_index = (integ_force_buffer_index + 1) % INTEG_FORCE_SAMPLES;
      float force_sum = 0;
      for (int i = 0; i < INTEG_FORCE_SAMPLES; i++) force_sum += integ_force_buffer[i];
      integ_current_force = force_sum / INTEG_FORCE_SAMPLES;
    }
  }
  
  // Force control update
  if (integ_force_control_active && 
      current_time - integ_last_force_control_update >= INTEG_FORCE_CONTROL_UPDATE_INTERVAL) {
    integ_last_force_control_update = current_time;
    integ_current_angle_id1 = getCurrentAngle();

    float error = integ_force_target - integ_current_force;
    float dt = INTEG_FORCE_CONTROL_UPDATE_INTERVAL / 1000.0;
    if (integ_force_last_control_time != 0) {
      dt = (current_time - integ_force_last_control_time) / 1000.0;
    }
    
    integ_force_integral_error += error * dt;
    integ_force_integral_error = constrainFloat(integ_force_integral_error, -5, 5);
    
    float derivative_error = 0;
    if (dt > 0) derivative_error = (error - integ_force_previous_error) / dt;
    
    float pid_output = integ_force_kp * error + integ_force_ki * integ_force_integral_error + integ_force_kd * derivative_error;
    
    // Use corrected Y coordinate
    float corrected_y = last_corrected_y;
    float feedforward_angle = 0.0;
    
    if (integ_robot_state == ROBOT_FORWARD || integ_robot_state == ROBOT_FORWARD_WAIT) {
        feedforward_angle = getFeedforwardAngle_Forward(corrected_y);
    } else {
        feedforward_angle = getFeedforwardAngle_Backward(corrected_y);
    }
    
    // float new_target_angle = feedforward_angle - pid_output;
    float new_target_angle = integ_current_angle_id1 - pid_output;
    new_target_angle = constrainFloat(new_target_angle, integ_force_min_angle, integ_force_max_angle);
    
    if (absFloat(new_target_angle - integ_target_angle_id1) > 0.1) {
      integ_target_angle_id1 = new_target_angle;
      moveToAngle(integ_target_angle_id1);
    }
    
    integ_force_previous_error = error;
    integ_force_last_control_time = current_time;
  }
  
  // Data log output
  if (integ_log_data && (current_time - integ_last_data_log_time >= INTEG_DATA_LOG_INTERVAL)) {
    integ_last_data_log_time = current_time;
    if (integ_robot_state == ROBOT_IDLE || integ_robot_state == ROBOT_CYCLE_COMPLETE) {
        if (integ_sensor_initialized) {
            myOtos.getVelocity(integ_current_velocity);
            integ_actual_speed = sqrt(integ_current_velocity.x * integ_current_velocity.x + 
                                       integ_current_velocity.y * integ_current_velocity.y) * 100.0;
            if (isnan(integ_actual_speed)) {
              integ_actual_speed = 0.0;
            }
        }
    }

    DEBUG_SERIAL.print("[LOG],");
    DEBUG_SERIAL.print(current_time);// Time [ms]
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(integ_current_force, 2);// Force [g]
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(last_corrected_y, 4); // Y coordinate [m]
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(last_corrected_x, 4); // X coordinate [m]
    DEBUG_SERIAL.print(",");
    float angle_deg = last_corrected_angle * 180.0 / PI;
    DEBUG_SERIAL.print(angle_deg, 2); // Robot heading angle [deg]
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.println(integ_actual_speed, 3); // Speed [cm/s]
  }
}

// Speed compensation helper functions
void integResetSpeedControl() {
  integ_speed_integral = 0.0;
  integ_speed_previous_error = 0.0;
  integ_speed_startup_counter = INTEG_SPEED_STARTUP_CYCLES;
  integ_last_velocity_control_time = millis();
  integ_actual_speed = 0.0;
  integ_speed_error = 0.0;
}

float integGetStartupSpeedScale() {
  if (integ_speed_startup_counter > 0) {
    float progress = 1.0 - ((float)integ_speed_startup_counter / (float)INTEG_SPEED_STARTUP_CYCLES);
    integ_speed_startup_counter--;
    return progress;
  }
  return 1.0;
}

float integCalculateSpeedCorrection(float dt) {
  if (!integ_velocity_control_enabled || !integ_sensor_initialized) return 0.0;
  if (dt <= 0.001) return 0.0;
  
  float target_speed_cms = integ_target_speed * 100.0;
  integ_speed_error = target_speed_cms - integ_actual_speed;
  
  if (integ_speed_startup_counter <= 5) {
    integ_speed_integral += integ_speed_error * dt * 0.1;
  } else {
    integ_speed_integral += integ_speed_error * dt;
  }
  integ_speed_integral = constrain(integ_speed_integral, -50.0, 50.0);
  
  float speed_derivative = 0.0;
  if (dt > 0) speed_derivative = (integ_speed_error - integ_speed_previous_error) / dt;
  
  float speed_correction = integ_speed_kp * integ_speed_error + 
                            integ_speed_ki * integ_speed_integral +
                            integ_speed_kd * speed_derivative;
  
  speed_correction = constrain(speed_correction, 
                               -integ_max_speed_correction, 
                               integ_max_speed_correction);
  
  integ_speed_previous_error = integ_speed_error;
  return speed_correction;
}

// ==========================================
// Robot control update (non-blocking wait implementation)
// ==========================================
void integUpdateRobotControl() {
  if (integ_stop_flag) {
    stopMovementMotorsReliable();
    integ_robot_is_running = false;
    integ_robot_state = ROBOT_IDLE;
    integ_log_data = false;
    integResetSpeedControl();
    DEBUG_SERIAL.println("\nReciprocating control interrupted");
    return;
  }
  
  // Speed measurement
  if (integ_sensor_initialized && integ_velocity_control_enabled) {
    myOtos.getVelocity(integ_current_velocity);
    integ_actual_speed = sqrt(integ_current_velocity.x * integ_current_velocity.x + 
                              integ_current_velocity.y * integ_current_velocity.y) * 100.0;
    if (isnan(integ_actual_speed)) integ_actual_speed = 0.0;
  }
  
  switch (integ_robot_state) {
    // ==========================================
    // Forward path (outbound)
    // ==========================================
    case ROBOT_FORWARD: {
      if (last_corrected_y >= integ_target_distance) {
        DEBUG_SERIAL.print("[FORWARD] Reached Y: ");
        DEBUG_SERIAL.println(last_corrected_y, 4);
        
        stopMovementMotorsReliable();
        
        integ_robot_state = ROBOT_FORWARD_WAIT;
        integ_last_robot_update = millis(); // Start wait timer
        integResetSpeedControl();
        DEBUG_SERIAL.println("[WAIT] Forward Stop -> Calibrating...");
      } else {

        // PID control during movement
        unsigned long current_time = millis();
        float dt = (current_time - integ_last_velocity_control_time) / 1000.0;
        if (dt <= 0.001) dt = integ_control_interval_ms / 1000.0;
        integ_last_velocity_control_time = current_time;
        float angle_correction = integ_angle_pid.update(last_corrected_angle);
        float speed_correction = 0.0;
        if (integ_velocity_control_enabled) {
          speed_correction = integCalculateSpeedCorrection(dt);
        }
        float left_speed = integ_base_speed - angle_correction + speed_correction;
        float right_speed = integ_base_speed + angle_correction + speed_correction;
        float startup_scale = integGetStartupSpeedScale();
        left_speed *= startup_scale;
        right_speed *= startup_scale;
        left_speed = constrain(left_speed, 0, 1023);
        right_speed = constrain(right_speed, 0, 1023);
        dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, (int)left_speed);
        dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, (int)right_speed | 0x400);
      }
      break;
    }
    
    // ==========================================
    // Forward path wait (preparing to turn around)
    // ==========================================
    case ROBOT_FORWARD_WAIT: {
      // Start switching process at wait begin (once only)
      if (!integ_scalar_switch_started) {
          otosHandleForwardToBackward_Start();
          integ_scalar_switch_started = true;
      }
      
      // After 500ms, run finish process and proceed
      if (millis() - integ_last_robot_update >= 500) {
        otosHandleForwardToBackward_Finish();
        integ_force_integral_error = 0.0;
        integ_robot_state = ROBOT_BACKWARD;
        integResetSpeedControl();
        integ_last_robot_update = millis();
        integ_scalar_switch_started = false;
        DEBUG_SERIAL.println("--- Backward Start ---");
      }
      break;
    }
    
    // ==========================================
    // Return path (backward)
    // ==========================================
    case ROBOT_BACKWARD: {
      if (last_corrected_y <= 0.0) {
        DEBUG_SERIAL.print("[BACKWARD] Reached Y: ");
        DEBUG_SERIAL.println(last_corrected_y, 4);
        
        stopMovementMotorsReliable();
        
        integ_robot_state = ROBOT_BACKWARD_WAIT;
        integ_last_robot_update = millis();
        integResetSpeedControl();
        DEBUG_SERIAL.println("[WAIT] Backward Stop -> Calibrating...");
      } else {
        // PID control during movement
        unsigned long current_time = millis();
        float dt = (current_time - integ_last_velocity_control_time) / 1000.0;
        if (dt <= 0.001) dt = integ_control_interval_ms / 1000.0;
        integ_last_velocity_control_time = current_time;
        float angle_correction = integ_angle_pid.update(last_corrected_angle);
        float speed_correction = 0.0;
        if (integ_velocity_control_enabled) {
          speed_correction = integCalculateSpeedCorrection(dt);
        }
        float left_speed = -(integ_base_speed + angle_correction + speed_correction);
        float right_speed = -(integ_base_speed - angle_correction + speed_correction);
        float startup_scale = integGetStartupSpeedScale();
        left_speed *= startup_scale;
        right_speed *= startup_scale;
        left_speed = constrain(left_speed, -1023, 0);
        right_speed = constrain(right_speed, -1023, 0);
        dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, (int)abs(left_speed) | 0x400);
        dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, (int)abs(right_speed));
      }
      break;
    }

    // ==========================================
    // Return path wait (cycle completion check)
    // ==========================================
    case ROBOT_BACKWARD_WAIT: {
      // Start switching process at wait begin
      if (!integ_scalar_switch_started) {
          otosHandleBackwardToForward_Start();
          integ_scalar_switch_started = true;
      }

      if (millis() - integ_last_robot_update >= 500) {
        otosHandleBackwardToForward_Finish();
        
        integ_current_cycle++;
        if (integ_current_cycle < integ_repeat_count) {
           integ_force_integral_error = 0.0;
           integ_robot_state = ROBOT_FORWARD;
           integResetSpeedControl();
           integ_last_robot_update = millis();
           integ_scalar_switch_started = false;
           
           DEBUG_SERIAL.print("=== Cycle ");
           DEBUG_SERIAL.print(integ_current_cycle + 1);
           DEBUG_SERIAL.println(" Start ===");
        } else {
           integ_robot_state = ROBOT_CYCLE_COMPLETE;
           integ_robot_is_running = false;
           integ_log_data = false;
           integResetSpeedControl();
           DEBUG_SERIAL.println("=== Round Trip Complete ===");
        }
      }
      break;
    }
    
    case ROBOT_CYCLE_COMPLETE:
    case ROBOT_IDLE:
      break;
  }
}

#endif // MODE_INTEGRATED_CONTROL