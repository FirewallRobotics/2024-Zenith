// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public CANSparkMax MasterIntakeMotor;

  public AbsoluteEncoder ArmEncoder;

  public DigitalInput intakeSensor;
  public DigitalOutput outputSensor;
  
  private double intakeSpeed = Constants.IntakeConstants.kIntakeMotorSpeed;
  private double indexSpeed = Constants.IntakeConstants.kIndexSpeed;
  private double indexReverseSpeed = Constants.IntakeConstants.kIndexReverseSpeed;

  public IntakeSubsystem() {
    MasterIntakeMotor =
        new CANSparkMax(IntakeConstants.kMasterIntakeMotorPort, MotorType.kBrushless);

    intakeSensor = new DigitalInput(IntakeConstants.kIntakeSensorPort);
    // outputSensor = new DigitalOutput(IntakeConstants.kIntakeOutputPort);
    // outputSensor.set(true);
    MasterIntakeMotor.restoreFactoryDefaults();

    // MasterIntakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    // MasterIntakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    // MasterIntakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);
    // MasterIntakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    // MasterIntakeMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    intakeSpeed = SmartDashboard.getNumber("Intake Speed", intakeSpeed);
    indexSpeed = SmartDashboard.getNumber("Index Speed", indexSpeed);
    indexReverseSpeed = SmartDashboard.getNumber("Index Reverse Speed", indexReverseSpeed);
  }

  /** Starts motor intake but stops if a note is detected inside */

  /** Starts motor intake */
  public void StartIntake() {
    MasterIntakeMotor.set(intakeSpeed);
  }

  public void StartIndex() {
    MasterIntakeMotor.set(indexSpeed);
  }

  public void StartReverseIndex() {
    MasterIntakeMotor.set(-indexReverseSpeed);
  }

  /** Stops motor intake */
  public void StopIntake() {
    MasterIntakeMotor.set(0);
  }
}
