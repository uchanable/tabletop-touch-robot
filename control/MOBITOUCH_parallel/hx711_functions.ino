#include "HX711.h"

void setupHX711Calibration() {
  Serial.begin(38400);
  Serial.println("=== HX711 Calibration ===");
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  
  Serial.println("Preparing...");
  delay(2000);
  
  // Step 1: Zero-point calibration
  Serial.println("\nStep 1: Zero-point calibration");
  Serial.println("Ensure nothing is touching the contact surface");
  Serial.println("Zero-point will be set automatically in 10 seconds...");
  
  for(int i = 10; i > 0; i--) {
    Serial.print(i);
    Serial.print("... ");
    delay(1000);
  }
  Serial.println();
  
  scale.tare();  // Set zero-point
  Serial.println("Zero-point calibration complete!");
  
  // Step 2: Scale calibration
  Serial.println("\nStep 2: Scale calibration");
  Serial.println("Adjust the stroking module height until the desired mass is reached (40g recommended)");
  Serial.println("Once the scale reading is stable, press any key and Enter");
}

void loopHX711Calibration() {
  // Calibration process
  static bool calibration_done = false;
  static bool waiting_for_input = true;
  static bool waiting_for_weight = false;
  
  if (!calibration_done && waiting_for_input) {
    // Waiting for user input
    if (Serial.available()) {
      while(Serial.available()) {
        Serial.read(); // Clear buffer
      }
      
      // Get measurement value
      long reading = scale.get_value(20);  // Average of 20 readings
      
      Serial.print("Reading: ");
      Serial.println(reading);
      Serial.println("\nEnter the adjusted mass in grams (e.g., 40):");
      
      waiting_for_input = false;
      waiting_for_weight = true;
    }
  }
  
  if (!calibration_done && waiting_for_weight) {
    // Waiting for weight input
    if (Serial.available()) {
      float known_weight = Serial.parseFloat();
      while(Serial.available()) {
        Serial.read(); // Clear buffer
      }
      
      if (known_weight > 0) {
        // Scale calculation
        long reading = scale.get_value(20);
        float calibration_factor = (float)reading / known_weight;
        
        Serial.print("Entered mass: ");
        Serial.print(known_weight);
        Serial.println("g");
        Serial.print("Calculated scale value: ");
        Serial.println(calibration_factor);
        
        // Scale setting
        scale.set_scale(calibration_factor);
        
        Serial.println("\nCalibration complete!");
        Serial.println("Use the following value in the main code:");
        Serial.print("scale.set_scale(");
        Serial.print(calibration_factor);
        Serial.println(");");
        
        Serial.println("\n=== Measurement Test ===");
        Serial.println("Change the weight to verify accuracy");
        
        calibration_done = true;
      }
    }
  }
  
  if (calibration_done) {
    // Continuous measurement
    if (scale.is_ready()) {
      float weight = scale.get_units(1);
      Serial.print("Mass: ");
      Serial.print(weight, 1);
      Serial.println(" g");
    }
    delay(100);
  }
}