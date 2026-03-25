#include <Servo.h>
#include <math.h>

Servo yawServo, shoulderServo, elbowServo;

int yawPin = 9;
int shoulderPin = 2;
int elbowPin = 10;

// Link lengths (change to your robot dimensions in mm)
float r1 = 74.63, r2 = 111.945, r3 = 130.05;

// --- Calibration angles ---
int yawHome = 0;
int shoulderHome = 0;
int elbowHome = 0;

void setup() {
  Serial.begin(9600);
  yawServo.attach(yawPin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);

  // --- Move to calibration position ---
  yawServo.write(yawHome);
  shoulderServo.write(shoulderHome);
  elbowServo.write(elbowHome);

  Serial.println("Calibration complete. Arm in home position.");
  Serial.println("Enter X Y Z (example: 100 50 120):");
}

void loop() {
  if (Serial.available()) {
    float x = Serial.parseFloat();
    float y = Serial.parseFloat();
    float z = Serial.parseFloat();

    if (Serial.read() == '\n') {  // wait for newline
      moveArm(x, y, z);
    }
  }
}

void solveAngles(float r, float z, float l2, float l3, float &theta2, float &theta3) {
    // Calculate intermediate values
    float K = (r*r + z*z + l3*l3 - l2*l2) / (2 * l3);
    float R = sqrt(r*r + z*z);
    
    // Check if solution exists
    if (fabs(K) > R) {
        theta2 = NAN;
        theta3 = NAN;
        return;
    }
    
    // Calculate theta3
    float alpha = atan2(z, r);
    float angle_diff = acos(K / R);
    theta3 = alpha + angle_diff;  // Using first solution
    
    // Calculate theta2
    float phi = atan2(z - l3*sin(theta3), r - l3*cos(theta3));
    theta2 = phi;
}

void moveArm(float x, float y, float z) {
  // --- Inverse Kinematics ---
  float theta1 = atan2(y, x);  // base yaw

  // Convert to cylindrical coordinates (r, z)
  float r = sqrt(x*x + y*y);
  float z_eff = z - r1;  // effective z coordinate
  
  // Use the solveAngles function to calculate theta2 and theta3
  float theta2, theta3;
  solveAngles(r, z_eff, r2, r3, theta2, theta3);
  
  // Check if solution exists
  if (isnan(theta2) || isnan(theta3)) {
    Serial.println("No solution exists for this position!");
    Serial.println("Please enter a different X Y Z coordinate.");
    return;
  }
  
  // --- Convert to degrees and constrain ---
  float deg1 = constrain(theta1 * 180.0 / PI, 0, 180);
  float deg2 = constrain(theta2 * 180.0 / PI, 0, 180);
  float deg3 = constrain(((PI/2)-theta3 + theta2) * 180.0 / PI, 0, 180);

  // --- Move servos ---
  yawServo.write(deg1);
  shoulderServo.write(deg2);
  elbowServo.write(deg3);

  // --- Debugging ---
  Serial.print("θ1 (Yaw): "); Serial.print(deg1, 1);
  Serial.print("°  θ2 (Shoulder): "); Serial.print(deg2, 1);
  Serial.print("°  θ3 (Elbow): "); Serial.print(deg3, 1);
  Serial.println("°");
  Serial.print("Target position - X: "); Serial.print(x, 1);
  Serial.print("mm  Y: "); Serial.print(y, 1);
  Serial.print("mm  Z: "); Serial.print(z, 1);
  Serial.println("mm");
  Serial.println("-----------------------------");
}
