package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSensor extends SubsystemBase {

  public DigitalOutput ultrasonicTrigger = new DigitalOutput(0);

  public AnalogInput ultrasonicSensor = new AnalogInput(0);

  public double ultrasonicSensorRange = 0;

  double voltage_scale_factor = 5 / RobotController.getVoltage5V();

  double currentDistanceCentimeters = ultrasonicSensor.getValue() * voltage_scale_factor * 0.125;

  double currentDistanceInches = ultrasonicSensor.getValue() * voltage_scale_factor * 0.0492;

  public UltrasonicSensor() {}

  @Override
  public void periodic() {
    currentDistanceCentimeters = ultrasonicSensor.getValue() * voltage_scale_factor * 0.125;

    SmartDashboard.putNumber("Sensor Range", currentDistanceCentimeters);
  }
}
