#if defined(MODE_FORCE_PROFILING)

#include "Wire.h"
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"

#ifndef DEBUG_SERIAL
#define DEBUG_SERIAL SerialUSB
#endif

// ==========================================
// Profile settings
// ==========================================
// ==========================================
// Odometry sensor calibration variables
// For dynamic phase, use the same values as in integrated_control.ino.
// For static phase, individual settings are needed (rarely drifted)
// ==========================================
// For static phase
const float OTOS_SCALAR_STATIC_FWD  = 0.940; // Static, forward path
const float OTOS_SCALAR_STATIC_BWD  = 0.930; // Static, return path
// For dynamic phase
const float OTOS_SCALAR_DYNAMIC_FWD = 0.935; // Dynamic, forward (set to same value as OTOS_FWD in integrated_control.ino)
const float OTOS_SCALAR_DYNAMIC_BWD = 1.020; // Dynamic, return (set to same value as OTOS_BWD in integrated_control.ino)

const float TURNAROUND_OFFSET_M = 0.005;

const float TARGET_FORCE_G = 40.0; // Target force [g]
const float TARGET_OVER_FORCE_G = 70.0; // Initial push force during static phase unloading [g]
const float FORCE_TOLERANCE_G = 3.0; // Allowable error during static phase [g]
const float DESCEND_SPEED_DEG_PER_LOOP = 3.0; // Angle step per unit until stroking module contacts the arm [deg]

// Stroking module control parameters for static phase 
const unsigned long FWD_MONOTONIC_WAIT_MS = 800;
const float FWD_MONOTONIC_STEP_DEG = 1.5; // Angle step per unit after stroking module contacts the arm [deg] (forward)
const unsigned long BWD_MONOTONIC_WAIT_MS = 800;
const float BWD_MONOTONIC_STEP_DEG = 1.5; // Angle step per unit after stroking module contacts the arm [deg] (backward)
const float BWD_CONTACT_FORCE_THRESHOLD = 10.0;   // Contact threshold of stroking module on arm [g]
const float POINT_DETECTION_THRESHOLD_M = 0.005; // Measurement point detection: +/-0.5cm
const float TOTAL_DISTANCE_M = 0.12;      // Total travel distance (12cm)
const float DENSE_INTERVAL_M = 0.03;      // Measurement point interval during dynamic phase (3cm)
const float STATIC_INTERVAL_M = 0.03;     // Measurement point interval during static phase (3cm)
const int MAX_PROFILE_POINTS = (int)(TOTAL_DISTANCE_M / DENSE_INTERVAL_M) + 1; // Number of measurement points

// Processing to avoid issues when changing the number of measurement points between dynamic and static phases
const int STATIC_STEP_INDEX = (int)(STATIC_INTERVAL_M / DENSE_INTERVAL_M);

// Motor range of motion for stroking module
const float MAX_MOTOR_ANGLE = 110.0;
const float MIN_MOTOR_ANGLE = -90.0;

const unsigned long ARM_MOVE_WAIT_MS = 2000;
const int MAX_ITERATIONS = 5; // Number of dynamic learning trials


// ==========================================
// Variables for accurate OTOS sensing
// ==========================================
float otos_y_offset = 0.0;
float otos_angle_offset = 0.0;
float last_corrected_angle = 0.0;
float last_corrected_x = 0.0;
float last_corrected_y = 0.0;
float temp_y_before_switch = 0.0;
float temp_angle_before_switch = 0.0;
bool prof_scalar_switch_started = false;

// ==========================================
// For data recording
// ==========================================
struct TrajectoryPoint {
  float position_y;
  float motor_angle;
};
TrajectoryPoint trajectory_data_forward[MAX_PROFILE_POINTS];
TrajectoryPoint trajectory_data_backward[MAX_PROFILE_POINTS];
int profile_count_forward = 0;
int profile_count_backward = 0;
float pid_corrections_fwd[MAX_PROFILE_POINTS];
float pid_corrections_bwd[MAX_PROFILE_POINTS];

float static_target_positions[MAX_PROFILE_POINTS];

// Measured angle recording (for learning)
float last_actual_angle_fwd[MAX_PROFILE_POINTS];
float last_actual_angle_bwd[MAX_PROFILE_POINTS];
float last_force_error_fwd[MAX_PROFILE_POINTS];
float last_force_error_bwd[MAX_PROFILE_POINTS];

// ==========================================
// Arrays for data history storage
// ==========================================
// Data immediately after static scan
float static_angle_fwd[MAX_PROFILE_POINTS];
float static_angle_bwd[MAX_PROFILE_POINTS];

// Data for each trial [trial count][point]
float history_angle_fwd[MAX_ITERATIONS][MAX_PROFILE_POINTS]; // Input angle
float history_force_fwd[MAX_ITERATIONS][MAX_PROFILE_POINTS]; // Measured force
float history_angle_bwd[MAX_ITERATIONS][MAX_PROFILE_POINTS];
float history_force_bwd[MAX_ITERATIONS][MAX_PROFILE_POINTS];

// For storing force data during static measurement
float static_force_fwd[MAX_PROFILE_POINTS];
float static_force_bwd[MAX_PROFILE_POINTS];

// ==========================================
// Global variables for control
// ==========================================
sfe_otos_pose2d_t current_pose; 
float base_angle_profiler = 0.0; 
IntegPIDController angle_pid_profiler(2000.0, 1150.0, 250.0, 0.0); // PID parameters for heading angle control
float base_speed_profiler = 198.0; // Target speed [pulse] = (target speed 3.0 [cm/s] / (motor no-load RPM 114 [rpm] / 60 [s] * wheel diameter 2.6*pi [cm] / 1024 [pulse]))
float current_target_angle = MAX_MOTOR_ANGLE;
IntegPIDController prof_robot_angle_pid(2000.0, 1150.0, 250.0, 0.0); // PID parameters for heading angle control
float prof_robot_base_speed = 198.0; // Target speed [pulse] = (target speed 3.0 [cm/s] / (motor no-load RPM 114 [rpm] / 60 [s] * wheel diameter 2.6*pi [cm] / 1024 [pulse]))
float prof_target_speed = 0.030; // Target speed [m/s]

// PID parameters for speed control
float prof_speed_kp = 100.0;
float prof_speed_ki = 70.0;
float prof_speed_kd = 0.3;

float prof_max_speed_correction = 1023.0; // Maximum speed correction amount
bool prof_velocity_control_enabled = true;
const int PROF_SPEED_STARTUP_CYCLES = 10; // Time to reach target speed 0.2[s] / control period 0.02[s]
sfe_otos_pose2d_t prof_current_velocity;
float prof_speed_integral = 0.0;
float prof_speed_previous_error = 0.0;
unsigned long prof_last_velocity_control_time = 0;
float prof_actual_speed = 0.0;
float prof_speed_error = 0.0;
int prof_speed_startup_counter = 0;

// PID parameters for force control
float force_kp = 0.3;
float force_ki = 0.03;
float force_kd = 0.03;
float force_integral = 0.0;
float force_prev_error = 0.0;
float prof_pid_output = 0.0;
float prof_last_target_angle = 0.0;
float prof_current_force = 0.0;
const int PROF_FORCE_SAMPLES = 5;
float prof_force_buffer[PROF_FORCE_SAMPLES];
int prof_force_buffer_index = 0;
unsigned long prof_last_force_read = 0;
const unsigned long PROF_FORCE_READ_INTERVAL = 20; // Load cell reading interval [ms]
bool hx711_available_prof = false;
bool otos_available_prof = false;
bool is_returning = false;
unsigned long state_timer = 0;
bool move_initialized = false;
float move_start_x = 0.0;
float move_start_y = 0.0;
int iteration_k = 0;
int current_dynamic_point = 0;
float pid_correction_accumulator = 0.0;
int pid_correction_count = 0;
const unsigned long prof_control_interval_ms = 20;// Control period [ms]
const unsigned long prof_force_control_interval_ms = 50;// Force control period [ms]

// For dynamic acceleration zone detection
float g_accel_settle_pos_m = 0.12;        // Acceleration completion point (initial value set to max distance)
bool  g_accel_reached = false;            // Reached flag
const float ACCEL_SETTLE_MARGIN_M = 0.05; // Margin after reaching (3cm)

unsigned long prof_last_robot_update = 0;
unsigned long prof_last_force_control_update = 0;
sfe_otos_pose2d_t prof_cycle_start_position;
float prof_target_distance = 0.0;
float static_target_y = 0.0;
bool prof_stop_flag = false;

// Robot state machine
enum ProfileState {
  PROF_IDLE,
  PROF_STATIC_MOVE_UP,
  PROF_STATIC_WAIT_FOR_MOVE_UP,
  PROF_STATIC_FIND_OVER_FORCE,
  PROF_STATIC_MONOTONIC_FWD,
  PROF_STATIC_FIND_CONTACT,
  PROF_STATIC_MONOTONIC_BWD,
  PROF_STATIC_RECORD_DATA,
  PROF_STATIC_MOVE_UP_AGAIN,
  PROF_STATIC_MOVE_ROBOT_STEP,
  PROF_DYNAMIC_START_ITERATION,
  PROF_DYNAMIC_RUN_INTEGRATED,
  PROF_DYNAMIC_APPLY_REFINEMENT,
  PROF_COMPLETE
};
ProfileState profile_state = PROF_IDLE;
enum ProfRobotState {
  PROF_ROBOT_IDLE,
  PROF_ROBOT_FORWARD,
  PROF_ROBOT_FORWARD_WAIT,
  PROF_ROBOT_BACKWARD,
  PROF_ROBOT_BACKWARD_WAIT,
  PROF_ROBOT_CYCLE_COMPLETE
};
ProfRobotState prof_robot_state = PROF_ROBOT_IDLE;

bool converged_fwd[MAX_PROFILE_POINTS];
bool converged_bwd[MAX_PROFILE_POINTS];

// ==========================================
// Forward declarations
// ==========================================
float getFeedforwardAngle_Forward(float current_y);
float getFeedforwardAngle_Backward(float current_y);
void synchronizeTurnaroundPoint();
void interpolateTrajectories();
void profUpdateForceControl_FF_FB(bool backward);
void profResetSpeedControl();
float profCalculateSpeedCorrection(float dt);
float profGetStartupSpeedScale();
void profUpdateRobotControl_Integrated(); 
void updateProfilingState();
void stopAll();
void calibrateOTOS();
void printTrajectoryData();
void stopMovementMotorsReliable();
void otosHandleBackwardToForward_Start();
void otosHandleBackwardToForward_Finish();
void otosHandleForwardToBackward_Start();
void otosHandleForwardToBackward_Finish();
void otosHandleBackwardToForward();
void otosHandleForwardToBackward();
void otosUpdateCorrectedPosition();
float linearInterpolate(float y, float y0, float y1, float value0, float value1);
void correctForwardTrajectory(TrajectoryPoint* original_data, TrajectoryPoint* corrected_data, int data_count, bool* is_outlier);
void correctBackwardTrajectory(TrajectoryPoint* original_data, TrajectoryPoint* corrected_data, int data_count, bool* is_outlier);
void synchronizeTurnaroundPoint() {
   DEBUG_SERIAL.println("\n[Sync] Turnaround synchronization SKIPPED (Offset Mode)");
}


// ==========================================
// Accurate OTOS sensing functions
// ==========================================
void otosUpdateCorrectedPosition() {
  myOtos.getPosition(current_pose);
  last_corrected_angle = current_pose.h;
  last_corrected_x = current_pose.x;
  // Add offset to maintain continuity
  last_corrected_y = current_pose.y + otos_y_offset;
}

// ---------------------------------------------------------
// Backward to forward transition process
// ---------------------------------------------------------
void otosHandleBackwardToForward_Start() {
  DEBUG_SERIAL.println("[OTOS] Backward->Forward: Starting calibration");
  otosUpdateCorrectedPosition(); 
  // Switch between dynamic/static modes
  if (profile_state >= PROF_DYNAMIC_START_ITERATION) {
    // Dynamic mode forward path
    myOtos.setLinearScalar(OTOS_SCALAR_DYNAMIC_FWD); 
    DEBUG_SERIAL.print("[Scalar] Set Dynamic FWD: ");
    DEBUG_SERIAL.println(OTOS_SCALAR_DYNAMIC_FWD, 4);
  } else {
    // Static mode forward path
    myOtos.setLinearScalar(OTOS_SCALAR_STATIC_FWD);
    DEBUG_SERIAL.print("[Scalar] Set Static FWD: ");
    DEBUG_SERIAL.println(OTOS_SCALAR_STATIC_FWD, 4);
  }  
  delay(100); 
}

void otosHandleBackwardToForward_Finish() {
  myOtos.getPosition(current_pose);
  float y_after_raw = current_pose.y;
  float h_after_raw = current_pose.h;

  // Correct drift (stop position - new raw coordinate)
  otos_y_offset = temp_y_before_switch - y_after_raw;
 
  prof_robot_angle_pid.reset();
  prof_robot_angle_pid.setSetpoint(base_angle_profiler);  
  otosUpdateCorrectedPosition();
  DEBUG_SERIAL.print("[OTOS] Current Y: ");
  DEBUG_SERIAL.println(last_corrected_y, 4);
}

// Blocking wrapper for static mode etc.
void otosHandleBackwardToForward() {
  // 1. Lock current position
  otosUpdateCorrectedPosition();
  temp_y_before_switch = last_corrected_y;
  temp_angle_before_switch = last_corrected_angle;
  
  // 2. Start process (including delay 100)
  otosHandleBackwardToForward_Start();
  
  // 3. Additional stabilization wait for safety
  delay(500); 
  
  // 4. Finish process
  otosHandleBackwardToForward_Finish();
}

// ---------------------------------------------------------
// Forward to backward transition process (2 stages + wrapper)
// ---------------------------------------------------------
void otosHandleForwardToBackward_Start() {
  DEBUG_SERIAL.println("[OTOS] Forward->Backward: Starting scale change");
  otosUpdateCorrectedPosition();

  // Switch between dynamic/static modes
  if (profile_state >= PROF_DYNAMIC_START_ITERATION) {
    // Dynamic mode return path
    myOtos.setLinearScalar(OTOS_SCALAR_DYNAMIC_BWD);
    DEBUG_SERIAL.print("[Scalar] Set Dynamic BWD: ");
    DEBUG_SERIAL.println(OTOS_SCALAR_DYNAMIC_BWD, 4);
  } else {
    // Static mode return path
    myOtos.setLinearScalar(OTOS_SCALAR_STATIC_BWD);
    DEBUG_SERIAL.print("[Scalar] Set Static BWD: ");
    DEBUG_SERIAL.println(OTOS_SCALAR_STATIC_BWD, 4);
  }
  delay(100); 
}

void otosHandleForwardToBackward_Finish() {
  myOtos.getPosition(current_pose);
  float y_after_raw = current_pose.y;
  float h_after_raw = current_pose.h;
  
  // Correct drift
  otos_y_offset = temp_y_before_switch - y_after_raw;
  
  prof_robot_angle_pid.reset();
  prof_robot_angle_pid.setSetpoint(base_angle_profiler);  
  otosUpdateCorrectedPosition();
  DEBUG_SERIAL.print("Offset Updated: ");
  DEBUG_SERIAL.println(otos_y_offset, 5);
}

// Blocking wrapper for static mode etc.
void otosHandleForwardToBackward() {
  // 1. Lock current position
  otosUpdateCorrectedPosition();
  temp_y_before_switch = last_corrected_y;
  temp_angle_before_switch = last_corrected_angle;
  
  // 2. Start process
  otosHandleForwardToBackward_Start();
  
  // 3. Additional wait
  delay(500);
  
  // 4. Finish process
  otosHandleForwardToBackward_Finish();
}



// ==========================================
// Setup
// ==========================================
void setupForceProfiling() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL && millis() < 3000);

  DEBUG_SERIAL.println("\n========================================");
  DEBUG_SERIAL.println("Force control trajectory profiling mode");
  DEBUG_SERIAL.println("========================================");

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
    scale.set_scale(-2538.48);
    delay(500);
    // Wait a moment after scale setting
    
    // Reset to zero by taring at startup
    DEBUG_SERIAL.println("Load cell initializing...");
    scale.tare();
    delay(100);
    DEBUG_SERIAL.println("Load cell tare complete (reset to 0g)");
    
    hx711_available_prof = true;
  } else {
    hx711_available_prof = false;
  }
  
  dxl.begin(1000000); 
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  delay(100);
  
  if (dxl.ping(MOTOR_ID)) {
    dxl.torqueOff(MOTOR_ID);
    dxl.setOperatingMode(MOTOR_ID, OP_POSITION);
    uint16_t cw_limit = 0;
    uint16_t ccw_limit = 1023;
    dxl.write(MOTOR_ID, 6, (uint8_t*)&cw_limit, 2);
    dxl.write(MOTOR_ID, 8, (uint8_t*)&ccw_limit, 2);
    dxl.torqueOn(MOTOR_ID);
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
  
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
  
  Wire.begin();
  if (!myOtos.begin()) {
    otos_available_prof = false;
  } else {
    myOtos.setLinearUnit(kSfeOtosLinearUnitMeters);
    myOtos.setAngularUnit(kSfeOtosAngularUnitRadians);
    sfe_otos_pose2d_t offset = {0, 0.02385, PI};
    myOtos.setOffset(offset);
    
    myOtos.setLinearScalar(OTOS_SCALAR_DYNAMIC_FWD);
    myOtos.setAngularScalar(0.991);
    
    myOtos.calibrateImu();
    delay(1000);
    myOtos.resetTracking();
    delay(100);
    myOtos.getPosition(current_pose);
    
    otos_y_offset = 0.0; // Initialization
    
    last_corrected_angle = current_pose.h;
    last_corrected_x = current_pose.x;
    last_corrected_y = current_pose.y;
    
    base_angle_profiler = current_pose.h;
    
    otos_available_prof = true;
  }

  for (int i = 0; i < PROF_FORCE_SAMPLES; i++) {
    prof_force_buffer[i] = 0.0;
  }

  DEBUG_SERIAL.println("\nCommands:");
  DEBUG_SERIAL.println("  p    - Start profiling");
  DEBUG_SERIAL.println("  stop - Emergency stop");
  DEBUG_SERIAL.println("  cal  - OTOS calibration");
}

// ==========================================
// Main loop
// ==========================================
void loopForceProfiling() {
  unsigned long current_time = millis();
  if (DEBUG_SERIAL.available()) {
    String cmd = DEBUG_SERIAL.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("p")) {
      if (profile_state == PROF_IDLE || profile_state == PROF_COMPLETE) {
        if (!hx711_available_prof || !otos_available_prof) {
          DEBUG_SERIAL.println("Error: Sensor initialization failed");
          return;
        }
        // ==========================================
        // Auto-calibration execution
        // ==========================================
        DEBUG_SERIAL.println("\n[Auto] Starting auto-calibration before profiling...");
        calibrateOTOS(); // Execute cal command content
        delay(500);      // Stabilization wait after completion
        DEBUG_SERIAL.println("[Auto] Calibration complete. Starting profiling.\n");
        // ==========================================
        profile_count_forward = 0;
        profile_count_backward = 0;
        is_returning = false;
        iteration_k = 0;
        force_integral = 0.0;
        force_prev_error = 0.0;
        
        myOtos.setLinearScalar(OTOS_SCALAR_STATIC_FWD);
        myOtos.resetTracking();
        delay(100);
        myOtos.getPosition(current_pose);
        
         otos_y_offset = 0.0; // Reset at start
        
        last_corrected_angle = current_pose.h;
        last_corrected_x = current_pose.x;
        last_corrected_y = current_pose.y;
        
        base_angle_profiler = current_pose.h;
        for (int i = 0; i < MAX_PROFILE_POINTS; i++) {
          static_target_positions[i] = DENSE_INTERVAL_M * i; // 0, -0.01, -0.02... (1cm intervals)
          
          converged_fwd[i] = false;
          converged_bwd[i] = false;
          
          // Important: Initial value set to retract angle to avoid malfunction when unmeasured points are 0
          trajectory_data_forward[i].motor_angle = MAX_MOTOR_ANGLE;
          trajectory_data_backward[i].motor_angle = MAX_MOTOR_ANGLE;
          
          // Also initialize Y coordinates
          trajectory_data_forward[i].position_y = static_target_positions[i];
          trajectory_data_backward[i].position_y = static_target_positions[i];

          static_force_fwd[i] = 0.0;
          static_force_bwd[i] = 0.0;
        }
        
        DEBUG_SERIAL.println("\nStatic profiling target coordinates:");
        for (int i = 0; i < MAX_PROFILE_POINTS; i++) {
          DEBUG_SERIAL.print("Point");
          DEBUG_SERIAL.print(i);
          DEBUG_SERIAL.print(": Y = ");
          DEBUG_SERIAL.println(static_target_positions[i], 4);
        }
        
        angle_pid_profiler.reset();
        angle_pid_profiler.setSetpoint(base_angle_profiler);
        prof_robot_angle_pid.reset();
        prof_robot_angle_pid.setSetpoint(base_angle_profiler);
        
        profResetSpeedControl(); 
        
        prof_stop_flag = false; 
        profile_state = PROF_STATIC_MOVE_UP; 
        
        state_timer = millis();
        move_initialized = false;
        current_dynamic_point = 0;
        pid_correction_accumulator = 0.0;
        pid_correction_count = 0;
        prof_last_robot_update = millis(); 
        prof_last_force_control_update = millis(); 
        prof_robot_state = PROF_ROBOT_IDLE;
        
        prof_scalar_switch_started = false;
      }
    } 
    else if (cmd.equalsIgnoreCase("stop")) {
      prof_stop_flag = true;
      stopAll();
    }
    else if (cmd.equalsIgnoreCase("cal")) {
      prof_stop_flag = true;
      calibrateOTOS();
    }
  }

  if (current_time - prof_last_force_read >= PROF_FORCE_READ_INTERVAL) {
    prof_last_force_read = current_time;
    if (hx711_available_prof && scale.is_ready()) {
      float new_force = scale.get_units(1); 
      prof_force_buffer[prof_force_buffer_index] = new_force;
      prof_force_buffer_index = (prof_force_buffer_index + 1) % PROF_FORCE_SAMPLES;
      float force_sum = 0;
      for (int i = 0; i < PROF_FORCE_SAMPLES; i++) force_sum += prof_force_buffer[i];
      prof_current_force = force_sum / PROF_FORCE_SAMPLES;
    }
  }

  if (profile_state != PROF_IDLE) {
    updateProfilingState();
  }
}

// ==========================================
// Reliable stop process
// ==========================================
void stopMovementMotorsReliable() {
  // Send 3 times consecutively
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
  
  delay(50);
}

void stopAll() {
  profile_state = PROF_IDLE;
  prof_robot_state = PROF_ROBOT_IDLE;
  prof_stop_flag = true;
  // Send 3 times consecutively
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
  dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
  
  delay(200);
}

void calibrateOTOS() {
  if (!otos_available_prof) {
    DEBUG_SERIAL.println("Error: OTOS sensor unavailable");
    return;
  }
  
  DEBUG_SERIAL.println("\n--- OTOS calibration start ---");
  stopAll();
  
  DEBUG_SERIAL.println("IMU calibrating...");
  myOtos.calibrateImu();
  delay(1000);
  
  DEBUG_SERIAL.println("Resetting tracking...");
  myOtos.resetTracking();
  delay(500);
  
  myOtos.getPosition(current_pose);
  
  otos_angle_offset = 0.0;
  otos_y_offset = 0.0; // Reset
  
  last_corrected_angle = current_pose.h;
  last_corrected_x = current_pose.x;
  last_corrected_y = current_pose.y;
  base_angle_profiler = current_pose.h;
  
  angle_pid_profiler.reset();
  angle_pid_profiler.setSetpoint(base_angle_profiler);
  prof_robot_angle_pid.reset();
  prof_robot_angle_pid.setSetpoint(base_angle_profiler);
  
  DEBUG_SERIAL.println("Calibration complete");
  DEBUG_SERIAL.print("Reference angle: ");
  DEBUG_SERIAL.print(base_angle_profiler, 4);
  DEBUG_SERIAL.println(" rad\n");
}

// ==========================================
// Profiling state machine
// ==========================================
void updateProfilingState() {
  unsigned long current_time = millis();
  if (prof_stop_flag && profile_state != PROF_IDLE) {
      stopAll();
      return;
  }

  if (profile_state == PROF_DYNAMIC_RUN_INTEGRATED) {
    if ((prof_robot_state != PROF_ROBOT_FORWARD_WAIT && prof_robot_state != PROF_ROBOT_BACKWARD_WAIT) &&
        current_time - prof_last_robot_update < prof_control_interval_ms) {
    } else {
      if (prof_robot_state != PROF_ROBOT_FORWARD_WAIT && prof_robot_state != PROF_ROBOT_BACKWARD_WAIT) {
        prof_last_robot_update = current_time;
      }
      profUpdateRobotControl_Integrated();
    }

    if (current_time - prof_last_force_control_update >= prof_force_control_interval_ms) {
      prof_last_force_control_update = current_time;
      profUpdateForceControl_FF_FB(is_returning);
    }
  }

  switch (profile_state) {
    
    case PROF_IDLE:
      break;
    case PROF_STATIC_MOVE_UP: {
      current_target_angle = MAX_MOTOR_ANGLE; 
      moveToAngle(current_target_angle);
      state_timer = current_time;
      profile_state = PROF_STATIC_WAIT_FOR_MOVE_UP;
      break;
    }
    
    case PROF_STATIC_WAIT_FOR_MOVE_UP: {
      if (current_time - state_timer > ARM_MOVE_WAIT_MS) {
        
        // Forward and backward path determination
        if (!is_returning) {
          // Forward path
          if (profile_count_forward == 0) {
            // Forward 1st point: Contact detection + push-in (same as backward)
            DEBUG_SERIAL.println("[Static] Forward Point 0 - Contact detection mode");
            current_target_angle = MAX_MOTOR_ANGLE;
            moveToAngle(current_target_angle);  // Execute
            
            // Wait for move to 150 degrees to complete
            delay(2000);
            // Wait 2 seconds for arm to fully stop
            
            profile_state = PROF_STATIC_FIND_CONTACT;
            state_timer = millis();
          } else {
            // Forward 2nd+ points: Start from 150 deg -> reach 50g -> unload
            DEBUG_SERIAL.print("[Static] Forward Point ");
            DEBUG_SERIAL.print(profile_count_forward);
            DEBUG_SERIAL.println(" - Unload mode (50g->40g)");
            
            current_target_angle = MAX_MOTOR_ANGLE;  // 150 degrees
            moveToAngle(current_target_angle);
            DEBUG_SERIAL.println("[Static] Moving to 150 degrees, then will enter FIND_OVER_FORCE");
            
            // Wait for move to 150 degrees to complete
            delay(2000);
            profile_state = PROF_STATIC_FIND_OVER_FORCE;
            state_timer = millis();
            DEBUG_SERIAL.print("[Static] State changed to PROF_STATIC_FIND_OVER_FORCE at ");
            DEBUG_SERIAL.println(millis());
          }
        } else {
          // All backward points: Contact detection + push-in
          DEBUG_SERIAL.print("[Static] Backward Point ");
          DEBUG_SERIAL.print(profile_count_backward);
          DEBUG_SERIAL.println(" - Contact detection mode");
          current_target_angle = MAX_MOTOR_ANGLE;
          moveToAngle(current_target_angle);
          // Execute
          
          // Wait for move to 150 degrees to complete
          delay(2000);
          profile_state = PROF_STATIC_FIND_CONTACT;
          state_timer = millis();
        }
      }
      break;
    }
    case PROF_STATIC_FIND_OVER_FORCE: {
      DEBUG_SERIAL.println("[Find Over Force] *** ENTERED FIND_OVER_FORCE STATE ***");
      if (prof_stop_flag) return; 
      if (!hx711_available_prof) { stopAll(); return; }

      float force = prof_current_force;
      DEBUG_SERIAL.print("[Find Over Force] F:");
      DEBUG_SERIAL.print(force, 1);
      DEBUG_SERIAL.print("g A:");
      DEBUG_SERIAL.println(current_target_angle, 1);
      if (force >= TARGET_OVER_FORCE_G) {
        DEBUG_SERIAL.println("[Find Over Force] *** 70g reached! Switch to monotonic control ***");
        force_integral = 0.0;
        force_prev_error = TARGET_FORCE_G - force;
        profile_state = PROF_STATIC_MONOTONIC_FWD;
        state_timer = millis();
      } else {
        current_target_angle -= DESCEND_SPEED_DEG_PER_LOOP;
        if (current_target_angle < MIN_MOTOR_ANGLE) {
          current_target_angle = MIN_MOTOR_ANGLE;
          DEBUG_SERIAL.print("[Find Over Force] *** Reached MIN angle but force only ");
          DEBUG_SERIAL.print(force, 1);
          DEBUG_SERIAL.println("g - Switch to monotonic anyway ***");
          profile_state = PROF_STATIC_MONOTONIC_FWD; 
          state_timer = millis();
        }
        if (prof_stop_flag) return; 
        moveToAngle(current_target_angle);
        delay(150);
      }
      break;
    }
    
    // ==========================================
    // Monotonic control (forward: after reaching 70g, during unloading, monotonic increase only)
    // ==========================================
    case PROF_STATIC_MONOTONIC_FWD: {
      if (!hx711_available_prof) { stopAll();
      return; }

      if (current_time - state_timer >= FWD_MONOTONIC_WAIT_MS) {
        state_timer = current_time;
        if (prof_stop_flag) return;

        float current_force = prof_current_force;
        float error = TARGET_FORCE_G - current_force;

        DEBUG_SERIAL.print("[Monotonic FWD] F:");
        DEBUG_SERIAL.print(current_force, 1);
        DEBUG_SERIAL.print("g E:");
        DEBUG_SERIAL.print(error, 1);
        DEBUG_SERIAL.print("g A:");
        DEBUG_SERIAL.println(current_target_angle, 1);

        if (current_force < TARGET_FORCE_G - FORCE_TOLERANCE_G) {
          // Force too low (< 37g) -> raise angle (unload, monotonic increase only)
          DEBUG_SERIAL.println("[Monotonic FWD] Force too LOW - Ascend to unload");
          current_target_angle += FWD_MONOTONIC_STEP_DEG;
          current_target_angle = constrainFloat(current_target_angle, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);
          if (prof_stop_flag) return;
          moveToAngle(current_target_angle);
          
          DEBUG_SERIAL.print("[Monotonic FWD] Ascend ");
          DEBUG_SERIAL.print(FWD_MONOTONIC_STEP_DEG, 1);
          DEBUG_SERIAL.println("°");
        } else if (current_force > TARGET_FORCE_G + FORCE_TOLERANCE_G) {
          // Force too high (> 43g) -> continue raising (continue unloading)
          DEBUG_SERIAL.println("[Monotonic FWD] Force too HIGH - Continue ascending to unload");
          current_target_angle += FWD_MONOTONIC_STEP_DEG;
          current_target_angle = constrainFloat(current_target_angle, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);
          if (prof_stop_flag) return;
          moveToAngle(current_target_angle);
          
          DEBUG_SERIAL.print("[Monotonic FWD] Ascend ");
          DEBUG_SERIAL.print(FWD_MONOTONIC_STEP_DEG, 1);
          DEBUG_SERIAL.println("°");
        } else {
          // Within range (37g-43g) -> success
          DEBUG_SERIAL.println("[Monotonic FWD] *** CONVERGED! ***");
          DEBUG_SERIAL.print("[Monotonic FWD] Final angle: ");
          DEBUG_SERIAL.println(current_target_angle, 2);
          profile_state = PROF_STATIC_RECORD_DATA;
        }
      }
      break;
    }

    // ==========================================
    // Backward: Contact detection (fast descent)
    // ==========================================
    case PROF_STATIC_FIND_CONTACT: {
      if (prof_stop_flag) return;
      if (!hx711_available_prof) { stopAll(); return; }

      float force = prof_current_force;
      
      DEBUG_SERIAL.print("[Find Contact] F:");
      DEBUG_SERIAL.print(force, 1);
      DEBUG_SERIAL.print("g A:");
      DEBUG_SERIAL.println(current_target_angle, 1);
      
      if (force >= BWD_CONTACT_FORCE_THRESHOLD) {
        // Contact detected -> transition to monotonic control
        DEBUG_SERIAL.println("[Find Contact] *** Contact detected! Switch to monotonic control ***");
        profile_state = PROF_STATIC_MONOTONIC_BWD;
        state_timer = millis();
      } else {
        // Not yet in contact -> fast descent
        current_target_angle -= DESCEND_SPEED_DEG_PER_LOOP;
        // Descend quickly in 3-degree steps
        if (current_target_angle < MIN_MOTOR_ANGLE) {
          current_target_angle = MIN_MOTOR_ANGLE;
          DEBUG_SERIAL.println("[Find Contact] *** Reached minimum angle ***");
          profile_state = PROF_STATIC_MONOTONIC_BWD;
          state_timer = millis();
        }
        if (prof_stop_flag) return;
        moveToAngle(current_target_angle);
        delay(150);
      }
      break;
    }

    // ==========================================
    // Monotonic control (backward: after contact, during push-in, monotonic decrease only)
    // ==========================================
    case PROF_STATIC_MONOTONIC_BWD: {
      if (!hx711_available_prof) { stopAll();
      return; }

      if (current_time - state_timer >= BWD_MONOTONIC_WAIT_MS) {
        state_timer = current_time;
        if (prof_stop_flag) return;

        float current_force = prof_current_force;
        float error = TARGET_FORCE_G - current_force;

        DEBUG_SERIAL.print("[Monotonic BWD] F:");
        DEBUG_SERIAL.print(current_force, 1);
        DEBUG_SERIAL.print("g E:");
        DEBUG_SERIAL.print(error, 1);
        DEBUG_SERIAL.print("g A:");
        DEBUG_SERIAL.println(current_target_angle, 1);

        if (current_force < TARGET_FORCE_G - FORCE_TOLERANCE_G) {
          // Force too low -> lower angle (push-in, monotonic decrease only)
          current_target_angle -= BWD_MONOTONIC_STEP_DEG;
          current_target_angle = constrainFloat(current_target_angle, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);
          
          if (prof_stop_flag) return;
          moveToAngle(current_target_angle);
          DEBUG_SERIAL.print("[Monotonic BWD] Descend ");
          DEBUG_SERIAL.print(BWD_MONOTONIC_STEP_DEG, 1);
          DEBUG_SERIAL.println("°");
        } else if (current_force > TARGET_FORCE_G + FORCE_TOLERANCE_G) {
          // Force too high -> overshot -> stop (never reverse)
          DEBUG_SERIAL.println("[Monotonic BWD] *** Overshot - STOP ***");
          DEBUG_SERIAL.print("[Monotonic BWD] Final angle: ");
          DEBUG_SERIAL.println(current_target_angle, 2);
          profile_state = PROF_STATIC_RECORD_DATA;
        } else {
          // Within range -> success
          DEBUG_SERIAL.println("[Monotonic BWD] *** CONVERGED! ***");
          DEBUG_SERIAL.print("[Monotonic BWD] Final angle: ");
          DEBUG_SERIAL.println(current_target_angle, 2);
          profile_state = PROF_STATIC_RECORD_DATA;
        }
      }
      break;
    }
    
    case PROF_STATIC_RECORD_DATA: {
      if (!otos_available_prof) { stopAll();
      return; }
      
      otosUpdateCorrectedPosition();
      
      DEBUG_SERIAL.print("[Static] Record ");
      DEBUG_SERIAL.print(is_returning ? "BWD" : "FWD");
      DEBUG_SERIAL.print(" Point ");
      DEBUG_SERIAL.print(is_returning ? profile_count_backward : profile_count_forward);
      DEBUG_SERIAL.print(" - Y: ");
      DEBUG_SERIAL.print(last_corrected_y, 4);
      DEBUG_SERIAL.print(", Angle: ");
      DEBUG_SERIAL.println(current_target_angle, 2);
      DEBUG_SERIAL.print(", Force: ");
      DEBUG_SERIAL.println(prof_current_force, 2);
      
      if (!is_returning) {
        // Forward path recording
        trajectory_data_forward[profile_count_forward].position_y = last_corrected_y;
        trajectory_data_forward[profile_count_forward].motor_angle = current_target_angle; 

        // Save measured force
        static_force_fwd[profile_count_forward] = prof_current_force;
        
        // +STATIC_STEP_INDEX instead of +1 (skip 3)
        profile_count_forward += STATIC_STEP_INDEX;
        
      } else {
        // Backward path recording
        trajectory_data_backward[profile_count_backward].position_y = last_corrected_y;
        trajectory_data_backward[profile_count_backward].motor_angle = current_target_angle;

        // Save measured force
        static_force_bwd[profile_count_backward] = prof_current_force;
        
        // +STATIC_STEP_INDEX instead of +1
        profile_count_backward += STATIC_STEP_INDEX;
      }
      
      // End detection and transition
      if (!is_returning && profile_count_forward >= MAX_PROFILE_POINTS) {
          is_returning = true;
          profile_state = PROF_STATIC_MOVE_UP_AGAIN; 
      } else if (is_returning && profile_count_backward >= MAX_PROFILE_POINTS) {
          // Backward complete -> run interpolation to fill gaps
          interpolateTrajectories();
          
          // Set counter to total point count for dynamic mode
          profile_count_forward = MAX_PROFILE_POINTS;
          profile_count_backward = MAX_PROFILE_POINTS;
          
          profile_state = PROF_DYNAMIC_START_ITERATION;
      } else {
          profile_state = PROF_STATIC_MOVE_UP_AGAIN;
      }
      break;
    }
    
    case PROF_STATIC_MOVE_UP_AGAIN: {
      if (prof_stop_flag) return;
      moveToAngle(MAX_MOTOR_ANGLE);
      state_timer = current_time;
      
      if (!is_returning) {
        // Forward path
        static_target_y = static_target_positions[profile_count_forward];
      } else {
        // Return path
        if (profile_count_backward == 0) {
           static_target_y = TOTAL_DISTANCE_M;
        } else {
           int target_index = MAX_PROFILE_POINTS - 1 - profile_count_backward;
           static_target_y = static_target_positions[target_index];
        }
      }
      
      DEBUG_SERIAL.print("[Static] Next target Y: ");
      DEBUG_SERIAL.println(static_target_y, 4);
      profile_state = PROF_STATIC_MOVE_ROBOT_STEP;
      break;
    }
    
    case PROF_STATIC_MOVE_ROBOT_STEP: {
      if (!otos_available_prof) { stopAll();
      return; }
      
      if (current_time - state_timer < ARM_MOVE_WAIT_MS) return;
      if (prof_stop_flag) {
        stopAll();
        return;
      }
      
      if (!move_initialized) {
        if (is_returning) {
          // Switch before static movement
          otosHandleForwardToBackward();
        }
        
        otosUpdateCorrectedPosition();
        move_start_x = last_corrected_x;
        move_start_y = last_corrected_y;
        
        DEBUG_SERIAL.print("[Static] Move start Y: ");
        DEBUG_SERIAL.print(move_start_y, 4);
        DEBUG_SERIAL.print(", Target Y: ");
        DEBUG_SERIAL.println(static_target_y, 4);
        
        angle_pid_profiler.reset(); 
        angle_pid_profiler.setSetpoint(base_angle_profiler);
        move_initialized = true;
        prof_last_robot_update = current_time; 
      }

      if (current_time - prof_last_robot_update >= prof_control_interval_ms) {
        prof_last_robot_update = current_time;
        if (prof_stop_flag) return; 

        otosUpdateCorrectedPosition();
        
        bool reached = false;
        if (!is_returning) {
          reached = (last_corrected_y >= static_target_y);
        } else {
          reached = (last_corrected_y <= static_target_y);
        }

        if (reached) {
          DEBUG_SERIAL.print("[Static] Reached - Current Y: ");
          DEBUG_SERIAL.print(last_corrected_y, 4);
          DEBUG_SERIAL.print(", Target Y: ");
          DEBUG_SERIAL.println(static_target_y, 4);
          
          // Send 3 times consecutively
          dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
          dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
          dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
          dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
          dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
          dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
          
          angle_pid_profiler.reset();
          
          profile_state = PROF_STATIC_MOVE_UP; 
          move_initialized = false;
        } else {
          if (prof_stop_flag) return; 

          float angle_correction = angle_pid_profiler.update(last_corrected_angle);
          float left_speed, right_speed;
          
          if (!is_returning) {
            left_speed = base_speed_profiler - angle_correction;
            right_speed = base_speed_profiler + angle_correction;
            left_speed = constrain(left_speed, 0, 400); 
            right_speed = constrain(right_speed, 0, 400);
            int left_vel = (int)left_speed;
            int right_vel = (int)right_speed | 0x400;

            if (prof_stop_flag) return; 
            dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, left_vel);
            if (prof_stop_flag) return; 
            dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, right_vel);
          } else {
            left_speed = -(base_speed_profiler + angle_correction);
            right_speed = -(base_speed_profiler - angle_correction);
            left_speed = constrain(left_speed, -400, 400); 
            right_speed = constrain(right_speed, -400, 400);
            int left_vel = (left_speed >= 0) ? (int)left_speed : ((int)abs(left_speed) | 0x400);
            int right_vel = (right_speed >= 0) ?
            ((int)right_speed | 0x400) : (int)abs(right_speed);
            
            if (prof_stop_flag) return; 
            dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, left_vel);
            if (prof_stop_flag) return; 
            dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, right_vel);
          }
        }
      }
      break;
    }
    
    case PROF_DYNAMIC_START_ITERATION: {
      
      // End detection
      
      if (iteration_k >= MAX_ITERATIONS) {
        printTrajectoryData();
        profile_state = PROF_COMPLETE;
        break;
      }
      
      iteration_k++;
      DEBUG_SERIAL.print("\n========================================\n");
      DEBUG_SERIAL.print("[Dynamic] Iteration ");
      DEBUG_SERIAL.print(iteration_k);
      DEBUG_SERIAL.println(" Start (Manual Reset Mode)");
      DEBUG_SERIAL.println("========================================");
      
      
      // 1. Retract arm (for safety)
      
      DEBUG_SERIAL.println("[Setup] Lifting arm...");
      moveToAngle(MAX_MOTOR_ANGLE); // Raise
      delay(2000); // Wait for movement completion
      
      
      // 2. Wait for manual adjustment
      
      DEBUG_SERIAL.println("\n>>> Align the robot to start position (Y=0) and press Enter <<<");
      while (!DEBUG_SERIAL.available()) {
        delay(10);
      }
      DEBUG_SERIAL.readStringUntil('\n'); 
      DEBUG_SERIAL.println("[Setup] Reseting coordinates...");

      
      // 3. Reset only sensor coordinates
      
      if (otos_available_prof) {
          
          myOtos.resetTracking(); // Just reset coordinates to (0,0,0)
          delay(500);             // Stabilization wait
          
          // Reset offset variables
          otos_y_offset = 0.0;
          otos_angle_offset = 0.0;
          
          // Reapply scalar
          myOtos.setLinearScalar(OTOS_SCALAR_DYNAMIC_FWD); 
          
          // Update current position
          otosUpdateCorrectedPosition();
          prof_cycle_start_position.x = last_corrected_x;
          prof_cycle_start_position.y = last_corrected_y;
          prof_cycle_start_position.h = last_corrected_angle;
          
          // Update reference angle
          base_angle_profiler = last_corrected_angle; 
          
          DEBUG_SERIAL.println("Coordinates reset to (0,0)");
      }

      // Variable initialization
      for (int i = 0; i < MAX_PROFILE_POINTS; i++) {
        pid_corrections_fwd[i] = 0.0;
        pid_corrections_bwd[i] = 0.0;
        last_actual_angle_fwd[i] = 0.0;
        last_actual_angle_bwd[i] = 0.0;
        last_force_error_fwd[i] = 999.0;
        last_force_error_bwd[i] = 999.0;
      }
      current_dynamic_point = 0; 
      pid_correction_accumulator = 0.0;
      pid_correction_count = 0;
      // Reset acceleration detection
      g_accel_reached = false;
      g_accel_settle_pos_m = prof_target_distance; // Treat as zone until the end if not reached (safety measure)
      
      prof_target_distance = static_target_positions[MAX_PROFILE_POINTS - 1];
      
      // PID reset
      prof_robot_angle_pid.reset(); 
      prof_robot_angle_pid.setSetpoint(base_angle_profiler);
      
      
      // 4. Force control warm-up (with slow descent added)
      
      if (hx711_available_prof) {
        float start_ff_angle = trajectory_data_forward[0].motor_angle; 
        
        // =========================================================
        // Slow descent process
        // =========================================================
        DEBUG_SERIAL.print("[Setup] Descending to start angle: ");
        DEBUG_SERIAL.println(start_ff_angle);
        
        float current_angle_setting = MAX_MOTOR_ANGLE; // Assuming currently at retract position
        float step = 0.5; // Movement per loop (smaller = slower)
        
        // Lower slowly from top to bottom
        for (float a = current_angle_setting; a >= start_ff_angle; a -= step) {
          moveToAngle(a);
          delay(10); // Speed adjustment (larger = slower)
          
          // Exit loop if about to overshoot start angle
          if (a <= start_ff_angle + step) break;
        }
        // Ensure final position at target angle
        moveToAngle(start_ff_angle);
        delay(500); // Wait for vibration to settle
        // =========================================================

        DEBUG_SERIAL.println("[Setup] Force Warm-up Start (2s)...");
        // =========================================================
        // Gain switching (stationary mode)
        // =========================================================
        float temp_ki_backup = force_ki; // Save original setting
        force_ki = 1.6;                  // Use strong integral to eliminate drift while stationary!
        // =========================================================
        // (B) Warm-up loop
        unsigned long warmup_start = millis();
        prof_force_buffer_index = 0;
        force_integral = 0.0; 
        
        while (millis() - warmup_start < 5000) {
           if (scale.is_ready()) {
             float val = scale.get_units(1);
             prof_force_buffer[prof_force_buffer_index] = val;
             prof_force_buffer_index = (prof_force_buffer_index + 1) % PROF_FORCE_SAMPLES;
           }
           float sum = 0;
           for(int i=0; i<PROF_FORCE_SAMPLES; i++) sum += prof_force_buffer[i];
           prof_current_force = sum / PROF_FORCE_SAMPLES;
           
           float error = TARGET_FORCE_G - prof_current_force;
           float dt = 0.02;
           force_integral += error * dt;
           force_integral = constrainFloat(force_integral, -10, 10);
           float derivative_error = (error - force_prev_error) / 0.05;
           force_prev_error = error;
           float pid_out = force_kp * error + force_ki * force_integral;
           
           float target = start_ff_angle - pid_out;
           target = constrainFloat(target, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);
           
           moveToAngle(target);
           delay(20);
        }

        // =========================================================
        // Gain switching (running mode)
        // =========================================================
        force_ki = 0.03;       // Disable integral just before running (avoid interfering with learning)
        force_integral = 0.0; // Reset accumulated integral (prevent runaway)
        
        // =========================================================

        
        force_prev_error = TARGET_FORCE_G - prof_current_force;
        DEBUG_SERIAL.print("[Setup] Warm-up Done. I-term: ");
        DEBUG_SERIAL.println(force_integral);
      } else {
         force_integral = 0.0;
         force_prev_error = 0.0;
      }
      profResetSpeedControl();
      // Start running
      is_returning = false;
      prof_robot_state = PROF_ROBOT_FORWARD; 
      profile_state = PROF_DYNAMIC_RUN_INTEGRATED; 
      prof_last_robot_update = millis(); 
      prof_last_force_control_update = millis(); 
      prof_scalar_switch_started = false;
      
      DEBUG_SERIAL.println("[Dynamic] GO!");
      break;
    }
    
    case PROF_DYNAMIC_RUN_INTEGRATED: {
      if (prof_robot_state == PROF_ROBOT_CYCLE_COMPLETE) {
        profile_state = PROF_DYNAMIC_APPLY_REFINEMENT;
      }
      break;
    }

case PROF_DYNAMIC_APPLY_REFINEMENT: {
      DEBUG_SERIAL.println("\n========================================");
      DEBUG_SERIAL.println("[Learning] Final: PID Delta(w/Deadzone) + Trust-Based Double Sweep");
      DEBUG_SERIAL.println("========================================");
      // Save current trial data to history
      int hist_idx = iteration_k - 1; // Because iteration_k starts from 1
      if (hist_idx >= 0 && hist_idx < MAX_ITERATIONS) {
        for (int i = 0; i < MAX_PROFILE_POINTS; i++) {
          // --- Forward ---
          history_angle_fwd[hist_idx][i] = trajectory_data_forward[i].motor_angle;
          // Force = (target - error). Unvisited points (0.0) set to 0.
          if (last_actual_angle_fwd[i] != 0.0) {
             history_force_fwd[hist_idx][i] = TARGET_FORCE_G - last_force_error_fwd[i];
          } else {
             history_force_fwd[hist_idx][i] = 0.0;
          }
          
          // --- Backward ---
          history_angle_bwd[hist_idx][i] = trajectory_data_backward[i].motor_angle;
          if (last_actual_angle_bwd[i] != 0.0) {
             history_force_bwd[hist_idx][i] = TARGET_FORCE_G - last_force_error_bwd[i];
          } else {
             history_force_bwd[hist_idx][i] = 0.0;
          }
        }
        DEBUG_SERIAL.print("[History] Saved data for iteration ");
        DEBUG_SERIAL.println(iteration_k);
      }
      const float LEARNING_RATE_VAL = 1.0;
      const float MAX_SINGLE_UPDATE_DEG = 30.0;
      const float CONSTRAINT_OFFSET = 0.0; 
      const float CONVERGENCE_LIMIT_G = 8.0; 
      const float DANGER_LIMIT_G = -2.0;

      // Learning deadzone (no update if within this error range)
      // To prevent overfitting and degradation of backward path
      const float LEARNING_DEADZONE_G = 5.0; 

      // Dynamic zone calculation: (90% reached position) + (6.5cm)
      // Margin set to 6.5cm to cover up to Point 4 (6.0cm)
      float dynamic_accel_zone_limit = g_accel_settle_pos_m + TOTAL_DISTANCE_M * (0.065 / 0.12); 
      DEBUG_SERIAL.print("[Solver] Dynamic Accel Zone Limit: ");
      DEBUG_SERIAL.print(dynamic_accel_zone_limit, 3);
      DEBUG_SERIAL.println("m");


      // --------------------------------------------------
      // [Phase 1] PID update (with deadzone)
      // --------------------------------------------------
      // --- Forward ---
      for (int i = 0; i < profile_count_forward; i++) {
        if (last_actual_angle_fwd[i] == 0.0) continue;
        
        // Deadzone check: Skip update if error is small
        if (abs(last_force_error_fwd[i]) < LEARNING_DEADZONE_G) {
            // Already well-tuned, leave unchanged (stabilization)
            continue; 
        }

        float diff = -last_actual_angle_fwd[i]; 
        if (abs(diff) > 30.0) continue; 
        float update_step = diff * LEARNING_RATE_VAL;
        update_step = constrainFloat(update_step, -MAX_SINGLE_UPDATE_DEG, MAX_SINGLE_UPDATE_DEG);
        trajectory_data_forward[i].motor_angle += update_step;
        trajectory_data_forward[i].motor_angle = constrainFloat(trajectory_data_forward[i].motor_angle, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);
      }

      // --- Backward ---
      for (int i = 0; i < profile_count_backward; i++) {
        if (last_actual_angle_bwd[i] == 0.0) continue;

        // Deadzone check: Prevent backward path degradation!
        if (abs(last_force_error_bwd[i]) < LEARNING_DEADZONE_G) {
            // Already well-tuned, leave unchanged
            continue; 
        }

        float diff = -last_actual_angle_bwd[i];
        if (abs(diff) > 30.0) continue;
        float update_step = diff * LEARNING_RATE_VAL;
        update_step = constrainFloat(update_step, -MAX_SINGLE_UPDATE_DEG, MAX_SINGLE_UPDATE_DEG);
        trajectory_data_backward[i].motor_angle += update_step;
        trajectory_data_backward[i].motor_angle = constrainFloat(trajectory_data_backward[i].motor_angle, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);
      }

      // --------------------------------------------------
      // [Phase 2] Double sweep with trust (dynamic exception applied)
      // --------------------------------------------------
      DEBUG_SERIAL.println("[Solver] Double Sweep with Dynamic Exception...");
      
      // --- Forward Pass 1: Ceiling Check (trim peaks) ---
      for (int i = profile_count_forward - 2; i >= 0; i--) {
        
        // Unconditional skip if within exception zone
        if (trajectory_data_forward[i].position_y <= dynamic_accel_zone_limit) {
            DEBUG_SERIAL.print("[Solver] Skip Ceiling at FWD_Point ");
            DEBUG_SERIAL.print(i);
            DEBUG_SERIAL.println(" (In Accel Zone)");
            continue; 
        }
        // Deadzone check: Skip update if error is small
        if (abs(last_force_error_fwd[i]) < LEARNING_DEADZONE_G) {
            // Already well-tuned, leave unchanged (stabilization)
            continue; 
        }
        float next_angle = trajectory_data_forward[i+1].motor_angle;
        float upper_limit = next_angle - CONSTRAINT_OFFSET;
        
        if (trajectory_data_forward[i].motor_angle > upper_limit) {
            bool i_am_danger = (last_force_error_fwd[i] < DANGER_LIMIT_G);
            bool next_is_trusted = (abs(last_force_error_fwd[i+1]) < CONVERGENCE_LIMIT_G);
            if (!i_am_danger && next_is_trusted) {
              trajectory_data_forward[i].motor_angle = upper_limit;
            }
        }
      }

      // --- Forward Pass 2: Floor Check (fill valleys) ---
      for (int i = 1; i < profile_count_forward; i++) {
        
        // Unconditional skip if within exception zone
        if (trajectory_data_forward[i].position_y <= dynamic_accel_zone_limit) {
            DEBUG_SERIAL.print("[Solver] Skip Floor at FWD_Point ");
            DEBUG_SERIAL.print(i);
            DEBUG_SERIAL.println(" (In Accel Zone)");
            continue;
        }
        // Deadzone check: Skip update if error is small
        if (abs(last_force_error_fwd[i]) < LEARNING_DEADZONE_G) {
            // Already well-tuned, leave unchanged (stabilization)
            continue; 
        }

        float prev_angle = trajectory_data_forward[i-1].motor_angle;
        float lower_limit = prev_angle + CONSTRAINT_OFFSET;
        
        if (trajectory_data_forward[i].motor_angle < lower_limit) {
            bool i_am_trusted = (abs(last_force_error_fwd[i]) < CONVERGENCE_LIMIT_G);
            bool prev_is_trusted = (abs(last_force_error_fwd[i-1]) < CONVERGENCE_LIMIT_G);
            
            if (i_am_trusted && !prev_is_trusted) {
              trajectory_data_forward[i-1].motor_angle = trajectory_data_forward[i].motor_angle - CONSTRAINT_OFFSET;
            } 
            else {
              trajectory_data_forward[i].motor_angle = lower_limit;
            }
        }
      }
      
      // --- Backward (no changes) ---
      for (int i = 1; i < profile_count_backward; i++) {
        // Deadzone check: Skip update if error is small
        if (abs(last_force_error_bwd[i]) < LEARNING_DEADZONE_G) {
            // Already well-tuned, leave unchanged (stabilization)
            continue; 
        }

          float prev_angle = trajectory_data_backward[i-1].motor_angle;
          float upper_limit = prev_angle - CONSTRAINT_OFFSET;
          if (trajectory_data_backward[i].motor_angle > upper_limit) {
            bool i_am_danger = (last_force_error_bwd[i] < DANGER_LIMIT_G);
            bool prev_is_trusted = (abs(last_force_error_bwd[i-1]) < CONVERGENCE_LIMIT_G);
            if (!i_am_danger && prev_is_trusted) trajectory_data_backward[i].motor_angle = upper_limit;
          }
      }
      for (int i = profile_count_backward - 2; i >= 0; i--) {

                // Deadzone check: Skip update if error is small
        if (abs(last_force_error_bwd[i]) < LEARNING_DEADZONE_G) {
            // Already well-tuned, leave unchanged (stabilization)
            continue; 
        }
          float next_angle = trajectory_data_backward[i+1].motor_angle;
          float lower_limit = next_angle + CONSTRAINT_OFFSET;
          if (trajectory_data_backward[i].motor_angle < lower_limit) {
            bool i_am_trusted = (abs(last_force_error_bwd[i]) < CONVERGENCE_LIMIT_G);
            bool next_is_trusted = (abs(last_force_error_bwd[i+1]) < CONVERGENCE_LIMIT_G);
            if (i_am_trusted && !next_is_trusted) trajectory_data_backward[i+1].motor_angle = trajectory_data_backward[i].motor_angle - CONSTRAINT_OFFSET;
            else trajectory_data_backward[i].motor_angle = lower_limit;
          }
      }
  
      synchronizeTurnaroundPoint();
      DEBUG_SERIAL.println("========================================\n");
      profile_state = PROF_DYNAMIC_START_ITERATION;
      break;
  }
    
    case PROF_COMPLETE: {
      stopAll();
      profile_state = PROF_IDLE;
      break;
    }
  }
}

// ==========================================
// Helper functions
// ==========================================

// Linear interpolation function
float linearInterpolate(float y, float y0, float y1, float value0, float value1) {
  if (abs(y1 - y0) < 0.0001) return value0;
  float t = (y - y0) / (y1 - y0);
  return value0 + t * (value1 - value0);
}

// Detect outliers in forward path and interpolate
void correctForwardTrajectory(TrajectoryPoint* original_data, TrajectoryPoint* corrected_data, int data_count, bool* is_outlier) {
  if (data_count <= 0) return;
  // Copy original data
  for (int i = 0; i < data_count; i++) {
    corrected_data[i] = original_data[i];
    is_outlier[i] = false;
  }
  
  // Pass 1: Detect outliers by comparing with previous point (forward: angle should increase)
  for (int i = 1; i < data_count; i++) {
    if (corrected_data[i].motor_angle < corrected_data[i - 1].motor_angle) {
      // Decreasing -> outlier
      is_outlier[i] = true;
    }
  }
  
  // Pass 2: Re-validate by comparing with last valid point
  bool changed = true;
  int max_iterations = data_count;
  // Prevent infinite loop
  int iteration = 0;
  
  while (changed && iteration < max_iterations) {
    changed = false;
    iteration++;
    
    for (int i = 1; i < data_count; i++) {
      if (is_outlier[i]) continue;
      // Already marked as outlier
      
      // Find previous valid point
      int prev_valid = -1;
      for (int j = i - 1; j >= 0; j--) {
        if (!is_outlier[j]) {
          prev_valid = j;
          break;
        }
      }
      
      // Compare with valid point
      if (prev_valid >= 0) {
        if (corrected_data[i].motor_angle < corrected_data[prev_valid].motor_angle) {
          // Decreasing from previous valid point -> outlier
          is_outlier[i] = true;
          changed = true;
        }
      }
    }
  }
  
  // Replace outliers with interpolated values
  for (int i = 0; i < data_count; i++) {
    if (is_outlier[i]) {
      // Find valid points before and after
      int prev_valid = -1;
      int next_valid = -1;
      
      // Search forward for valid point
      for (int j = i - 1; j >= 0; j--) {
        if (!is_outlier[j]) {
          prev_valid = j;
          break;
        }
      }
      
      // Search backward for valid point
      for (int j = i + 1; j < data_count; j++) {
        if (!is_outlier[j]) {
          next_valid = j;
          break;
        }
      }
      
      // Interpolation
      if (prev_valid >= 0 && next_valid >= 0) {
        // Valid points on both sides
        float y0 = corrected_data[prev_valid].position_y;
        float y1 = corrected_data[next_valid].position_y;
        float angle0 = corrected_data[prev_valid].motor_angle;
        float angle1 = corrected_data[next_valid].motor_angle;
        float target_y = corrected_data[i].position_y;
        corrected_data[i].motor_angle = linearInterpolate(target_y, y0, y1, angle0, angle1);
      } else if (prev_valid >= 0) {
        // Only forward is valid
        corrected_data[i].motor_angle = corrected_data[prev_valid].motor_angle;
      } else if (next_valid >= 0) {
        // Only backward is valid
        corrected_data[i].motor_angle = corrected_data[next_valid].motor_angle;
      }
    }
  }
}

// Detect outliers in return path and interpolate
void correctBackwardTrajectory(TrajectoryPoint* original_data, TrajectoryPoint* corrected_data, int data_count, bool* is_outlier) {
  if (data_count <= 0) return;
  // Copy original data
  for (int i = 0; i < data_count; i++) {
    corrected_data[i] = original_data[i];
    is_outlier[i] = false;
  }
  
  // Pass 1: Detect outliers by comparing with previous point (backward: angle should decrease)
  for (int i = 1; i < data_count; i++) {
    if (corrected_data[i].motor_angle > corrected_data[i - 1].motor_angle) {
      // Increasing -> outlier
      is_outlier[i] = true;
    }
  }
  
  // Pass 2: Re-validate by comparing with last valid point
  bool changed = true;
  int max_iterations = data_count;
  // Prevent infinite loop
  int iteration = 0;
  
  while (changed && iteration < max_iterations) {
    changed = false;
    iteration++;
    
    for (int i = 1; i < data_count; i++) {
      if (is_outlier[i]) continue;
      // Already marked as outlier
      
      // Find previous valid point
      int prev_valid = -1;
      for (int j = i - 1; j >= 0; j--) {
        if (!is_outlier[j]) {
          prev_valid = j;
          break;
        }
      }
      
      // Compare with valid point
      if (prev_valid >= 0) {
        if (corrected_data[i].motor_angle > corrected_data[prev_valid].motor_angle) {
          // Increasing from previous valid point -> outlier
          is_outlier[i] = true;
          changed = true;
        }
      }
    }
  }
  
  // Replace outliers with interpolated values
  for (int i = 0; i < data_count; i++) {
    if (is_outlier[i]) {
      // Find valid points before and after
      int prev_valid = -1;
      int next_valid = -1;
      
      // Search forward for valid point
      for (int j = i - 1; j >= 0; j--) {
        if (!is_outlier[j]) {
          prev_valid = j;
          break;
        }
      }
      
      // Search backward for valid point
      for (int j = i + 1; j < data_count; j++) {
        if (!is_outlier[j]) {
          next_valid = j;
          break;
        }
      }
      
      // Interpolation
      if (prev_valid >= 0 && next_valid >= 0) {
        // Valid points on both sides
        float y0 = corrected_data[prev_valid].position_y;
        float y1 = corrected_data[next_valid].position_y;
        float angle0 = corrected_data[prev_valid].motor_angle;
        float angle1 = corrected_data[next_valid].motor_angle;
        float target_y = corrected_data[i].position_y;
        corrected_data[i].motor_angle = linearInterpolate(target_y, y0, y1, angle0, angle1);
      } else if (prev_valid >= 0) {
        // Only forward is valid
        corrected_data[i].motor_angle = corrected_data[prev_valid].motor_angle;
      } else if (next_valid >= 0) {
        // Only backward is valid
        corrected_data[i].motor_angle = corrected_data[next_valid].motor_angle;
      }
    }
  }
}


void printTrajectoryData() {
  DEBUG_SERIAL.println("\n========================================");
  DEBUG_SERIAL.println("=== Final Learning Results Output ===");
  DEBUG_SERIAL.println("========================================");
  DEBUG_SERIAL.println("\n--- Final Learned Data ---");
  
  DEBUG_SERIAL.println("\nForward path data:");
  for (int i = 0; i < profile_count_forward; i++) {
    DEBUG_SERIAL.print("{");
    DEBUG_SERIAL.print(trajectory_data_forward[i].position_y, 4);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(trajectory_data_forward[i].motor_angle, 2);
    DEBUG_SERIAL.println("},");
  }
  
  DEBUG_SERIAL.println("\nBackward path data:");
  for (int i = 0; i < profile_count_backward; i++) {
    DEBUG_SERIAL.print("{");
    DEBUG_SERIAL.print(trajectory_data_backward[i].position_y, 4);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(trajectory_data_backward[i].motor_angle, 2);
    DEBUG_SERIAL.println("},");
  }

  // ==========================================
  // 2. Forward data output for Excel analysis
  // ==========================================
  DEBUG_SERIAL.println("\n========================================");
  DEBUG_SERIAL.println("=== Final Data Output (Vertical Format) ===");
  DEBUG_SERIAL.println("========================================");
  
  DEBUG_SERIAL.println("\n=== FORWARD DATA START ===");

  // --- Static measurement data ---
  DEBUG_SERIAL.println("Static measurement data");
  DEBUG_SERIAL.println("point, angle, force");
  for (int i = 0; i < profile_count_forward; i++) {
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(static_angle_fwd[i], 2);
    DEBUG_SERIAL.print(", ");
    // Output measured array
    DEBUG_SERIAL.println(static_force_fwd[i], 2); 
  }
  DEBUG_SERIAL.println(); // Blank line

  // --- Dynamic measurement data (loop for all trials) ---
  for (int k = 0; k < MAX_ITERATIONS; k++) {
    DEBUG_SERIAL.print("Dynamic trial ");
    DEBUG_SERIAL.print(k + 1);
    DEBUG_SERIAL.println(" data");
    DEBUG_SERIAL.println("point, angle, force");

    for (int i = 0; i < profile_count_forward; i++) {
      DEBUG_SERIAL.print(i);
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.print(history_angle_fwd[k][i], 2);
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.println(history_force_fwd[k][i], 2);
    }
    DEBUG_SERIAL.println(); 
  }

  // ==========================================
  // 3. Backward data output for Excel analysis
  // ==========================================
  DEBUG_SERIAL.println("\n=== BACKWARD DATA START ===");

  // --- Static measurement data ---
  DEBUG_SERIAL.println("Static measurement data");
  DEBUG_SERIAL.println("point, angle, force");
  for (int i = 0; i < profile_count_backward; i++) {
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(", ");
    DEBUG_SERIAL.print(static_angle_bwd[i], 2);
    DEBUG_SERIAL.print(", ");
    // Output measured array
    DEBUG_SERIAL.println(static_force_bwd[i], 2);
  }
  DEBUG_SERIAL.println();

  // --- Dynamic measurement data (loop for all trials) ---
  for (int k = 0; k < MAX_ITERATIONS; k++) {
    DEBUG_SERIAL.print("Dynamic trial ");
    DEBUG_SERIAL.print(k + 1);
    DEBUG_SERIAL.println(" data");
    DEBUG_SERIAL.println("point, angle, force");

    for (int i = 0; i < profile_count_backward; i++) {
      DEBUG_SERIAL.print(i);
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.print(history_angle_bwd[k][i], 2);
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.println(history_force_bwd[k][i], 2);
    }
    DEBUG_SERIAL.println();
  }

  DEBUG_SERIAL.println("========================================");
  DEBUG_SERIAL.println("=== Data Output Complete ===");
  DEBUG_SERIAL.println("========================================\n");
}
// ==========================================
// Trajectory interpolation function
// ==========================================
float getFeedforwardAngle_Forward(float current_y) { 
  if (MAX_PROFILE_POINTS <= 1) return trajectory_data_forward[0].motor_angle;

  // If before start point (0.0), return start point angle
  if (current_y <= trajectory_data_forward[0].position_y) {
    return trajectory_data_forward[0].motor_angle;
  }
  
  // If beyond goal point (0.12), return goal point angle
  float last_y = trajectory_data_forward[MAX_PROFILE_POINTS - 1].position_y;
  if (current_y >= last_y) {
    return trajectory_data_forward[MAX_PROFILE_POINTS - 1].motor_angle;
  }
  
  for (int i = 0; i < MAX_PROFILE_POINTS - 1; i++) {
    float y0 = trajectory_data_forward[i].position_y;
    float y1 = trajectory_data_forward[i+1].position_y;
    
    // Check if within increasing range
    if (current_y >= y0 && current_y <= y1) {
      float a0 = trajectory_data_forward[i].motor_angle;
      float a1 = trajectory_data_forward[i+1].motor_angle;
      
      if (abs(y1 - y0) < 0.0001) return a0;
      
      // Linear interpolation
      return a0 + (current_y - y0) * (a1 - a0) / (y1 - y0);
    }
  }
  
  return trajectory_data_forward[MAX_PROFILE_POINTS - 1].motor_angle;
}

float getFeedforwardAngle_Backward(float current_y) {
  if (MAX_PROFILE_POINTS <= 1) return trajectory_data_backward[0].motor_angle;

  // If beyond turnaround point (0.12), return first angle
  if (current_y >= trajectory_data_backward[0].position_y) {
    return trajectory_data_backward[0].motor_angle;
  }
  
  // If before return point (0.0), return last angle
  float last_y = trajectory_data_backward[MAX_PROFILE_POINTS - 1].position_y;
  if (current_y <= last_y) {
    return trajectory_data_backward[MAX_PROFILE_POINTS - 1].motor_angle;
  }
  
  for (int i = 0; i < MAX_PROFILE_POINTS - 1; i++) {
    float y0 = trajectory_data_backward[i].position_y; // Larger value (e.g., 0.12)
    float y1 = trajectory_data_backward[i+1].position_y; // Smaller value (e.g., 0.11)
    
    // Check if within decreasing range
    // (y0 >= current >= y1)
    if (current_y <= y0 && current_y >= y1) {
      float a0 = trajectory_data_backward[i].motor_angle;
      float a1 = trajectory_data_backward[i+1].motor_angle;
      
      if (abs(y1 - y0) < 0.0001) return a0;
      
      // Linear interpolation
      return a0 + (current_y - y0) * (a1 - a0) / (y1 - y0);
    }
  }
  
  return trajectory_data_backward[MAX_PROFILE_POINTS - 1].motor_angle;
}

// ==========================================
// Integrated control function
// ==========================================
void profUpdateRobotControl_Integrated() {
  if (prof_stop_flag) {
    dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
    dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
    prof_robot_state = PROF_ROBOT_IDLE;
    profResetSpeedControl();
    return; 
  }
  
  otosUpdateCorrectedPosition();
  
  if (prof_velocity_control_enabled) {
      myOtos.getVelocity(prof_current_velocity);
      prof_actual_speed = sqrt(prof_current_velocity.x * prof_current_velocity.x + 
                               prof_current_velocity.y * prof_current_velocity.y) * 100.0;
      if (isnan(prof_actual_speed)) {
        prof_actual_speed = 0.0;
      }
  }


  // Monitor if 90% of target speed is reached (forward only)
  if (prof_robot_state == PROF_ROBOT_FORWARD && !g_accel_reached) {
      float target_speed_cms = prof_target_speed * 100.0;
      // If speed exceeds 90%
      if (prof_actual_speed >= (target_speed_cms * 0.9)) {
          g_accel_settle_pos_m = last_corrected_y; // Record current position
          g_accel_reached = true;
          DEBUG_SERIAL.print("[AutoZone] Speed 90% reached at: ");
          DEBUG_SERIAL.print(g_accel_settle_pos_m, 4);
          DEBUG_SERIAL.println("m");
      }
  }

  if (prof_robot_state == PROF_ROBOT_FORWARD || prof_robot_state == PROF_ROBOT_BACKWARD) {
      float current_y = last_corrected_y;
      // Calculate distance to each measurement point and record nearest 
      if (!is_returning) {
        // Forward path: Record when passing each measurement point
        for (int i = 0; i < MAX_PROFILE_POINTS; i++) {
          float target_y = trajectory_data_forward[i].position_y;
          float distance = abs(current_y - target_y);
          
          // Within specified range of measurement point and not yet recorded
          if (distance < POINT_DETECTION_THRESHOLD_M && last_actual_angle_fwd[i] == 0.0) {
            
            // Record PID correction amount as-is
            last_actual_angle_fwd[i] = prof_pid_output;
            
            // Record force error
            last_force_error_fwd[i] = TARGET_FORCE_G - prof_current_force;

            DEBUG_SERIAL.print("[Actual] FWD Point ");
            DEBUG_SERIAL.print(i);
            DEBUG_SERIAL.print(": Err="); // Force error
            DEBUG_SERIAL.print(last_force_error_fwd[i], 1);
            DEBUG_SERIAL.print("g, InAngle="); // Input angle (absolute)
            DEBUG_SERIAL.print(prof_last_target_angle, 2);
            DEBUG_SERIAL.print(", PIDCorr="); // Learning data (correction amount)
            DEBUG_SERIAL.println(prof_pid_output, 2);
          }
        }
      } else {
        // Return path: Record when passing each measurement point
        for (int i = 0; i < MAX_PROFILE_POINTS; i++) {
          float target_y = trajectory_data_backward[i].position_y;
          float distance = abs(current_y - target_y);
          
          // Within specified range of measurement point and not yet recorded
          if (distance < POINT_DETECTION_THRESHOLD_M && last_actual_angle_bwd[i] == 0.0) {
            
            // Record PID correction amount as-is
            last_actual_angle_bwd[i] = prof_pid_output;
            
            // Record force error
            last_force_error_bwd[i] = TARGET_FORCE_G - prof_current_force;
            
            DEBUG_SERIAL.print("[Actual] BWD Point ");
            DEBUG_SERIAL.print(i);
            DEBUG_SERIAL.print(": Err="); // Force error
            DEBUG_SERIAL.print(last_force_error_bwd[i], 1);
            DEBUG_SERIAL.print("g, InAngle="); // Input angle (absolute)
            DEBUG_SERIAL.print(prof_last_target_angle, 2);
            DEBUG_SERIAL.print(", PIDCorr="); // Learning data (correction amount)
            DEBUG_SERIAL.println(prof_pid_output, 2);
          }
        }
      }
      
      
      pid_correction_accumulator += prof_pid_output;
      pid_correction_count++;
  }

  switch (prof_robot_state) {
    case PROF_ROBOT_FORWARD: {
      if (last_corrected_y >= prof_target_distance) {
        DEBUG_SERIAL.print("[Dynamic] Forward reached - Y: ");
        DEBUG_SERIAL.print(last_corrected_y, 4);
        DEBUG_SERIAL.print(", Target: ");
        DEBUG_SERIAL.println(prof_target_distance, 4);
        
        // Send 3 times consecutively
        dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
        dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
        dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
        dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
        dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
        dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
        
        // Lock coordinates
        temp_y_before_switch = last_corrected_y;
        temp_angle_before_switch = last_corrected_angle;
        
        prof_robot_state = PROF_ROBOT_FORWARD_WAIT;
        prof_last_robot_update = millis(); 
        profResetSpeedControl();
        
        // Flag initialization
        prof_scalar_switch_started = false;
      } else {
        unsigned long current_time = millis();
        float dt = (current_time - prof_last_velocity_control_time) / 1000.0;
        if (dt <= 0.001) dt = prof_control_interval_ms / 1000.0;
        prof_last_velocity_control_time = current_time;
        float angle_correction = prof_robot_angle_pid.update(last_corrected_angle);
        float speed_correction = 0.0;
        
        if (prof_velocity_control_enabled) {
          speed_correction = profCalculateSpeedCorrection(dt);
        }
        
        float left_speed = prof_robot_base_speed - angle_correction + speed_correction;
        float right_speed = prof_robot_base_speed + angle_correction + speed_correction;
        
        float startup_scale = profGetStartupSpeedScale();
        left_speed *= startup_scale;
        right_speed *= startup_scale;
        left_speed = constrain(left_speed, 0, 1023); 
        right_speed = constrain(right_speed, 0, 1023);
        
        int left_vel = (int)left_speed;
        int right_vel = (int)right_speed | 0x400;
        if (prof_stop_flag) return; 
        dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, left_vel);
        if (prof_stop_flag) return; 
        dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, right_vel);
      }
      break;
    }

    // ==========================================
    // Forward wait (non-blocking wait using millis())
    // ==========================================
    case PROF_ROBOT_FORWARD_WAIT: {
      unsigned long elapsed = millis() - prof_last_robot_update;
      
      // Phase 1: Start scalar change after 200ms
      if (elapsed >= 200 && !prof_scalar_switch_started) {
        if (pid_correction_count > 0) {
          float avg_correction = pid_correction_accumulator / pid_correction_count;
          pid_corrections_fwd[current_dynamic_point] = avg_correction;
        }
        
        otosHandleForwardToBackward_Start();
        prof_scalar_switch_started = true;
      }
      
      // Phase 2: Complete after additional 500ms (total 700ms)
      if (elapsed >= 700) {
        otosHandleForwardToBackward_Finish();
      
        profResetSpeedControl();


        // =========================================================
        // Fully reset I-term before starting return path
        // =========================================================
        force_integral = 0.0; 
        force_prev_error = 0.0;
        
        // The integral value from end of forward path is just noise for return path,
        //    so discard it here and re-accumulate from zero
        DEBUG_SERIAL.println("[Dynamic] Force Integral Reset for Backward");
        // =========================================================

        prof_robot_state = PROF_ROBOT_BACKWARD; 
        prof_last_robot_update = millis();
        is_returning = true;
        
        otosUpdateCorrectedPosition();
        prof_cycle_start_position.x = last_corrected_x;
        prof_cycle_start_position.y = last_corrected_y;
        prof_cycle_start_position.h = last_corrected_angle;
        prof_target_distance = static_target_positions[0];
        
        DEBUG_SERIAL.print("[Dynamic] Backward start Y: ");
        DEBUG_SERIAL.print(prof_cycle_start_position.y, 4);
        DEBUG_SERIAL.print(", Target: ");
        DEBUG_SERIAL.println(prof_target_distance, 4);
        
        pid_correction_accumulator = 0.0;
        pid_correction_count = 0;
        current_dynamic_point = MAX_PROFILE_POINTS - 1;
      }
      break;
    }

    case PROF_ROBOT_BACKWARD: {
      if (last_corrected_y <= prof_target_distance) {
        DEBUG_SERIAL.print("[Dynamic] Backward reached - Y: ");
        DEBUG_SERIAL.print(last_corrected_y, 4);
        DEBUG_SERIAL.print(", Target: ");
        DEBUG_SERIAL.println(prof_target_distance, 4);
        
        // Send 3 times consecutively
        dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
        dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
        dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
        dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
        dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, 0);
        dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, 0);
        
        // Lock coordinates
        temp_y_before_switch = last_corrected_y;
        temp_angle_before_switch = last_corrected_angle;
        
        prof_robot_state = PROF_ROBOT_BACKWARD_WAIT;
        prof_last_robot_update = millis();
        profResetSpeedControl();
        
        prof_scalar_switch_started = false;
      } else {
        unsigned long current_time = millis();
        float dt = (current_time - prof_last_velocity_control_time) / 1000.0;
        if (dt <= 0.001) dt = prof_control_interval_ms / 1000.0;
        prof_last_velocity_control_time = current_time;
        float angle_correction = prof_robot_angle_pid.update(last_corrected_angle);
        float speed_correction = 0.0;
        
        if (prof_velocity_control_enabled) {
          speed_correction = profCalculateSpeedCorrection(dt);
        }
        
        float left_speed = -(prof_robot_base_speed + angle_correction + speed_correction);
        float right_speed = -(prof_robot_base_speed - angle_correction + speed_correction);
        
        float startup_scale = profGetStartupSpeedScale();
        left_speed *= startup_scale;
        right_speed *= startup_scale;
        left_speed = constrain(left_speed, -1023, 0); 
        right_speed = constrain(right_speed, -1023, 0);
        
        int left_vel = (int)abs(left_speed) | 0x400;
        int right_vel = (int)abs(right_speed);
        if (prof_stop_flag) return; 
        dxl.setGoalVelocity(INTEG_LEFT_MOTOR_ID, left_vel);
        if (prof_stop_flag) return; 
        dxl.setGoalVelocity(INTEG_RIGHT_MOTOR_ID, right_vel);
      }
      break;
    }

    // ==========================================
    // Backward wait (non-blocking wait using millis())
    // ==========================================
    case PROF_ROBOT_BACKWARD_WAIT: {
      unsigned long elapsed = millis() - prof_last_robot_update;
      
      // Phase 1: Start scalar change after 200ms
      if (elapsed >= 200 && !prof_scalar_switch_started) {
        if (pid_correction_count > 0) {
          float avg_correction = pid_correction_accumulator / pid_correction_count;
          pid_corrections_bwd[current_dynamic_point] = avg_correction;
        }
        otosHandleBackwardToForward_Start();
        prof_scalar_switch_started = true;
      }
      
      // Phase 2: Complete after 700ms
      if (elapsed >= 700) {
        otosHandleBackwardToForward_Finish();
        force_integral = 0.0;
        force_prev_error = 0.0;
        DEBUG_SERIAL.println("[Dynamic] Force Integral Reset for Forward");
        prof_robot_state = PROF_ROBOT_CYCLE_COMPLETE;
        is_returning = false;
      }
      break;
    }

    case PROF_ROBOT_CYCLE_COMPLETE:
    case PROF_ROBOT_IDLE:
      break;
  }
}

// ==========================================
// Force control function
// ==========================================
void profUpdateForceControl_FF_FB(bool backward) {
  if (prof_stop_flag) return;
  if (!hx711_available_prof) return;

  float current_force = prof_current_force;
  float error = TARGET_FORCE_G - current_force;

  float dt = prof_force_control_interval_ms / 1000.0;
  force_integral += error * dt;
  force_integral = constrainFloat(force_integral, -5, 5);
  
  float derivative_error = (error - force_prev_error) / dt;
  force_prev_error = error;
  prof_pid_output = force_kp * error + force_ki * force_integral + force_kd * derivative_error;
  
  float current_y = last_corrected_y;
  float ff_angle;
  if (!backward) {
    ff_angle = getFeedforwardAngle_Forward(current_y);
  } else {
    ff_angle = getFeedforwardAngle_Backward(current_y);
  }

  float target_angle = ff_angle - prof_pid_output; 
  target_angle = constrainFloat(target_angle, MIN_MOTOR_ANGLE, MAX_MOTOR_ANGLE);
  prof_last_target_angle = target_angle;
  if (prof_stop_flag) return;
  moveToAngle(target_angle);
}

// ==========================================
// Speed control helper
// ==========================================
void profResetSpeedControl() {
  prof_speed_integral = 0.0;
  prof_speed_previous_error = 0.0;
  prof_last_velocity_control_time = millis();
  prof_speed_startup_counter = PROF_SPEED_STARTUP_CYCLES;
  prof_actual_speed = 0.0;
  prof_speed_error = 0.0;
}

float profGetStartupSpeedScale() {
  if (prof_speed_startup_counter > 0) {
    float progress = 1.0 - ((float)prof_speed_startup_counter / (float)PROF_SPEED_STARTUP_CYCLES);
    prof_speed_startup_counter--;
    return progress;
  }
  return 1.0;
}

float profCalculateSpeedCorrection(float dt) {
  if (dt <= 0.0) return 0.0;
  float target_speed_cms = prof_target_speed * 100.0;
  prof_speed_error = target_speed_cms - prof_actual_speed;
  if (prof_speed_startup_counter <= 5) {
    prof_speed_integral += prof_speed_error * dt * 0.1;
  } else {
    prof_speed_integral += prof_speed_error * dt;
  }
  prof_speed_integral = constrain(prof_speed_integral, -50.0, 50.0);
  float speed_derivative = 0.0;
  if (dt > 0) {
    speed_derivative = (prof_speed_error - prof_speed_previous_error) / dt;
  }
  
  float speed_correction = prof_speed_kp * prof_speed_error + 
                           prof_speed_ki * prof_speed_integral +
                           prof_speed_kd * speed_derivative;
  speed_correction = constrain(speed_correction, 
                               -prof_max_speed_correction, 
                               prof_max_speed_correction);
  prof_speed_previous_error = prof_speed_error;
  
  return speed_correction;
}

// ==========================================
// Trajectory data linear interpolation function
// ==========================================
void interpolateTrajectories() {
  DEBUG_SERIAL.println("\n[Interpolation] Filling gaps in trajectory (Angle AND Position)... Force gaps set to 0.0");

  // --- Forward path interpolation ---
  for (int i = 0; i < MAX_PROFILE_POINTS - 1; i += STATIC_STEP_INDEX) {
    int next = i + STATIC_STEP_INDEX;
    if (next >= MAX_PROFILE_POINTS) break; 

    // Get start point and end point
    float angle0 = trajectory_data_forward[i].motor_angle;
    float angle1 = trajectory_data_forward[next].motor_angle;
    float pos0 = trajectory_data_forward[i].position_y;
    float pos1 = trajectory_data_forward[next].position_y;

    // Calculate intermediate points
    for (int j = i + 1; j < next; j++) {
      float progress = (float)(j - i) / (float)(next - i);
      
      // Linearly interpolate angle and position (needed for control)
      trajectory_data_forward[j].motor_angle = angle0 + (angle1 - angle0) * progress;
      trajectory_data_forward[j].position_y = pos0 + (pos1 - pos0) * progress;
      
      // Do not interpolate force, set to 0.0 (indicates unmeasured)
      static_force_fwd[j] = 0.0;
    }
  }

  // --- Return path interpolation ---
  for (int i = 0; i < MAX_PROFILE_POINTS - 1; i += STATIC_STEP_INDEX) {
    int next = i + STATIC_STEP_INDEX;
    if (next >= MAX_PROFILE_POINTS) break;

    float angle0 = trajectory_data_backward[i].motor_angle;
    float angle1 = trajectory_data_backward[next].motor_angle;
    float pos0 = trajectory_data_backward[i].position_y;
    float pos1 = trajectory_data_backward[next].position_y;

    for (int j = i + 1; j < next; j++) {
      float progress = (float)(j - i) / (float)(next - i);
      
      trajectory_data_backward[j].motor_angle = angle0 + (angle1 - angle0) * progress;
      trajectory_data_backward[j].position_y = pos0 + (pos1 - pos0) * progress;

      // Do not interpolate force, set to 0.0
      static_force_bwd[j] = 0.0;
    }
  }
  
  // Backup of static angle data
  for(int i = 0; i < MAX_PROFILE_POINTS; i++){
    static_angle_fwd[i] = trajectory_data_forward[i].motor_angle;
    static_angle_bwd[i] = trajectory_data_backward[i].motor_angle;
  }

  DEBUG_SERIAL.println("[Interpolation] Done with Position fix. Static data saved.");
}

#endif // MODE_FORCE_PROFILING