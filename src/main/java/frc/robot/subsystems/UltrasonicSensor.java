// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.UltrasonicConstants;

public class UltrasonicSensor extends SubsystemBase {

  // Gets the output of the sensor. High output when object is detected in range
  public DigitalOutput ultrasonicTrigger =
      new DigitalOutput(UltrasonicConstants.kUltrasonicTriggerPort);

  // Create an instance of the AnalogInput class so we can read from it later
  public AnalogInput ultrasonicSensor = new AnalogInput(UltrasonicConstants.kUltrasonicSensorPort);

  // Ultrasonic range. How far an object was from the sensor. The closer the object, the higher the
  // mm.
  /*Targets closer than 30-cm will typically range as 300-mm.
   *
   *objects between 30-cm and 50-cm may experience acoustic phase
   *cancellation of the returning waveform resulting in inaccuracies of up to 5-mm.
   */
  public double ultrasonicSensorRange = 0;

  // voltage_scale_factor allows us to compensate for differences in supply voltage
  double voltage_scale_factor = 5 / RobotController.getVoltage5V();

  // Formula to calculate range in Centimeters:
  double currentDistanceCentimeters = ultrasonicSensor.getValue() * voltage_scale_factor * 0.125;

  // Calculate into inches
  double currentDistanceInches = ultrasonicSensor.getValue() * voltage_scale_factor * 0.0492;

  private boolean rangeOf30 = false;

  /** Creates a new UltrasonicSensor. */
  public UltrasonicSensor() {}

  @Override
  public void periodic() {
    currentDistanceCentimeters = ultrasonicSensor.getValue() * voltage_scale_factor * 0.125;

    SmartDashboard.putNumber("Sensor Range", currentDistanceCentimeters);

    if (currentDistanceCentimeters <= 30) {
      rangeOf30 = true;
    } else {
      rangeOf30 = false;
    }
  }

  public double speedNeeded() {
    double precentOfSpeed;
    if (rangeOf30 == true) {
      precentOfSpeed = percentFormual(currentDistanceCentimeters, 30);
      return precentOfSpeed;
    } else {
      return DriveConstants.kMaxSpeedMetersPerSecond;
    }
  }

  public boolean inRange30() {
    if (rangeOf30) {
      return true;
    } else {
      return false;
    }
  }

  private double percentFormual(double part, double whole) {
    double percent;
    if (whole != 0 && part != 0) {
      percent = (part / whole);
      return percent;
    } else {
      return 0;
    }
  }
}
