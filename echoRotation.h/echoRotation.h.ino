/*
  The code is released under the GNU General Public License.
  Developed by www.codekraft.it
*/

#include "CurieIMU.h"

#include "pitches.h"

// notes in the melody:
int melody[] = {
  NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C6
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 4, 4, 8
};


int pitch;
int maxPitch = 1047;
int minPitch = 65;

bool triggered = false;
float target_x = 120;
float target_y = 120;
float target_z = 120;// sound position
unsigned long target_spawn_time;
unsigned long reactionTime;

// IMU
float FS_SEL = 131;                                  // IMU gyro values to degrees/sec
unsigned long last_read_time;
float angle_x, angle_y, angle_z;                     // These are the result angles
float last_x_angle, last_y_angle, last_z_angle;      // These are the filtered angles
float lGXA, lGYA, lGZA;                              // Store the gyro angles to compare drift

// FUNCTIONS
//Math
//Convert radians to degrees
double rtod(double fradians) {
  return (fradians * 180.0 / PI);
}
#define runEvery(t) for (static long _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))
void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x; last_y_angle = y; last_z_angle = z;
  lGXA = x_gyro; lGYA = y_gyro; lGZA = z_gyro;
}

void setup() {
  target_spawn_time = millis();
  Serial.begin(9600);
  //while (!Serial);

  // init CurieIMU
  CurieIMU.begin();
  // use the code below to calibrate accel/gyro offset values
  Serial.println("Internal sensor offsets BEFORE calibration...");
  Serial.print(CurieIMU.getXAccelOffset());
  Serial.print("\t"); // -76
  Serial.print(CurieIMU.getYAccelOffset());
  Serial.print("\t"); // -235
  Serial.print(CurieIMU.getZAccelOffset());
  Serial.print("\t"); // 168
  Serial.print(CurieIMU.getXGyroOffset());
  Serial.print("\t"); // 0
  Serial.print(CurieIMU.getYGyroOffset());
  Serial.print("\t"); // 0
  Serial.println(CurieIMU.getZGyroOffset());
  Serial.println("About to calibrate. Make sure your board is stable and upright");
  delay(1000);
  // The board must be resting in a horizontal position for
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration...");
  CurieIMU.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration...");
  CurieIMU.autoCalibrateXAccelOffset(0);
  CurieIMU.autoCalibrateYAccelOffset(0);
  CurieIMU.autoCalibrateZAccelOffset(1);
  Serial.println(" Done");
  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieIMU.setGyroOffsetEnabled(true);
  CurieIMU.setAccelOffsetEnabled(true);

  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
}

void loop() {
  unsigned long t_now = millis();
  int ax = CurieIMU.getAccelerationX();
  int ay = CurieIMU.getAccelerationY();
  int az = CurieIMU.getAccelerationZ();
  int gx = CurieIMU.getRotationX();
  int gy = CurieIMU.getRotationY();
  int gz = CurieIMU.getRotationZ();


  // Convert gyro values to degrees/sec
  float gyro_x = gx / FS_SEL;
  float gyro_y = gy / FS_SEL;
  float gyro_z = gz / FS_SEL;

  // Compute the accel angles
  float accelX = rtod(atan(ay / sqrt( pow(ax, 2) + pow(az, 2))));
  float accelY = rtod(-1 * atan(ax / sqrt(pow(ay, 2) + pow(az, 2))));

  // Compute the (filtered) gyro angles
  float dt = (t_now - last_read_time) / 1000.0;
  float gyroX = gyro_x * dt + last_x_angle;
  float gyroY = gyro_y * dt + last_y_angle;
  float gyroZ = gyro_z * dt + last_z_angle;

  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x * dt + lGXA;
  float unfiltered_gyro_angle_y = gyro_y * dt + lGYA;
  float unfiltered_gyro_angle_z = gyro_z * dt + lGZA;

  // Apply the complementary filter to figure out the change in angle
  // Alpha depends on the sampling rate...
  float alpha = 0.96;
  angle_x = alpha * gyroX + (1.0 - alpha) * accelX;
  angle_y = alpha * gyroY + (1.0 - alpha) * accelY;
  angle_z = gyroZ;  //Accelerometer doesn't give z-angle

  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

  /*Serial.print("Y:" );
    Serial.print(angle_y);
    Serial.print("  \t Z:" );
    Serial.print(angle_z);
    Serial.print("  \t X:" );
    Serial.println(angle_x);
  */

  //calculates delta from target angles
  float dx = target_x - angle_x;
  float dy = target_y - angle_y;
  float delta = sqrt(pow(dx, 2) + pow(dy, 2));
  
  //maps deltas to chosen pitches
  pitch = map(constrain(delta, 75, 240), 240, 100, minPitch, maxPitch);
  Serial.println(pitch);
  
  tone(9, pitch);
  
  //calculates the time it takes to acheive find target
  reactionTime = millis() - target_spawn_time;

  //the longer it takes to find the maximum pitch, the less accurate you need to be (in order to speed up the demos)
  if (pitch > (maxPitch - ((int)reactionTime)/100) && !triggered)
  {
    triggered = true;

    //post reaction time
    Serial.print("reaction time: ");
    Serial.print(reactionTime / 1000);
    Serial.println("s");
    delay(100);

    //play reward sound
    for (int thisNote = 0; thisNote < 4; thisNote++) {

      // to calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / noteDurations[thisNote];
      tone(9, melody[thisNote], noteDuration);

      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 0.50;
      delay(pauseBetweenNotes);
    }
    //generate new target
    moveTargets();
  }
}

void moveTargets() {
  target_x = random(100, 240);
  target_y = random(100, 240);
  target_z = 0;
  //set the timer to compare the reaction time to
  target_spawn_time = millis();
  triggered = false;
}

