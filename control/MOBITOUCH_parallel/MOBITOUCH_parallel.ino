// ==========================================
// Select the mode to run (uncomment only one)
// ==========================================

// #define MODE_HX711_CALIBRATION    // HX711 calibration
// #define MODE_FORCE_PROFILING      // *1. Create trajectory map
#define MODE_INTEGRATED_CONTROL   // *2. Run this after copying the trajectory map

// ==========================================
// Global variables and timing control
// ==========================================
unsigned long current_time = 0;
unsigned long last_update = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("=== MOBITOUCH Project (FF+FB compatible version) ===");

#ifdef MODE_HX711_CALIBRATION
  Serial.println("Mode: HX711 Calibration");
  setupHX711Calibration();

#elif defined(MODE_FORCE_PROFILING)
  Serial.println("Mode: *Force Control Trajectory Profiling");
  setupForceProfiling(); // Output starts on SerialUSB

#elif defined(MODE_INTEGRATED_CONTROL)
  Serial.println("Mode: *Integrated Control (FF+FB)");
  setupIntegratedControl(); // Output starts on SerialUSB
  
#else
  Serial.println("Error: No mode selected");
  Serial.println("Please uncomment a MODE_xxx definition in setup()");
#endif

  Serial.println("Initialization complete");
  Serial.println("========================");
}

void loop() {
  current_time = millis();
  // Update every 50ms
  if (current_time - last_update >= 50) {
    last_update = current_time;

#ifdef MODE_HX711_CALIBRATION
    loopHX711Calibration();

#endif
  }

  // Robot control runs directly in the main loop
#if defined(MODE_INTEGRATED_CONTROL)
  loopIntegratedControl();
#elif defined(MODE_FORCE_PROFILING)
  loopForceProfiling();
#endif
  
}