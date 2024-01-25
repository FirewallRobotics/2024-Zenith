// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Import required WPILib libraries (Ultrasonic Sensor)
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DigitalOutput;

//Robot libraries
//import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class UltrasonicSensor extends SubsystemBase {


  //Gets the output of the sensor. High output when object is detected in range
  public DigitalOutput ultrasonicTrigger = new DigitalOutput(0);

  //Create an instance of the AnalogInput class so we can read from it later
  public AnalogInput ultrasonicSensor = new AnalogInput(0);

  //Ultrasonic range. How far an object was from the sensor. The closer the object, the higher the mm.
  /*Targets closer than 30-cm will typically range as 300-mm.
  *
  *objects between 30-cm and 50-cm may experience acoustic phase
  *cancellation of the returning waveform resulting in inaccuracies of up to 5-mm.
  */
  public double ultrasonicSensorRange = 0;

  //voltage_scale_factor allows us to compensate for differences in supply voltage
  double voltage_scale_factor = 5/RobotController.getVoltage5V();

  //Formula to calculate range in Centimeters:
  double currentDistanceCentimeters = ultrasonicSensor.getValue() * voltage_scale_factor * 0.125;

  //Calculate into inches
  double currentDistanceInches = ultrasonicSensor.getValue() * voltage_scale_factor * 0.0492;




  /** Creates a new UltrasonicSensor. */
  public UltrasonicSensor() {}



  

  

  @Override
  public void periodic() {
  currentDistanceCentimeters = ultrasonicSensor.getValue() * voltage_scale_factor * 0.125;

 
  SmartDashboard.putNumber("Sensor Range", currentDistanceCentimeters);

  }


 



  
}
