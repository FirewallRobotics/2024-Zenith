// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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
  public DigitalInput outputSensor;
  public DigitalOutput NoteDetectedLED;
  public DigitalOutput NoteReadyLED;

  StringLogEntry speedOfIntake;

  private double intakeSpeed = Constants.IntakeConstants.kIntakeMotorSpeed;
  private double indexSpeed = Constants.IntakeConstants.kIndexSpeed;
  private double indexReverseSpeed = Constants.IntakeConstants.kIndexReverseSpeed;

  public boolean noteInShooter = false;

  public IntakeSubsystem() {
    MasterIntakeMotor =
        new CANSparkMax(IntakeConstants.kMasterIntakeMotorPort, MotorType.kBrushless);

    intakeSensor = new DigitalInput(IntakeConstants.kIntakeSensorPort);
    outputSensor = new DigitalInput(IntakeConstants.kIntakeOutputPort);

    NoteDetectedLED = new DigitalOutput(IntakeConstants.kNoteDetectedLEDPort);
    NoteReadyLED = new DigitalOutput(IntakeConstants.kNoteReadyLEDPort);
    NoteDetectedLED.set(false);
    NoteReadyLED.set(false);

    MasterIntakeMotor.restoreFactoryDefaults();
    DataLog log = DataLogManager.getLog();
    speedOfIntake = new StringLogEntry(log, "Speed of Intake");

    // MasterIntakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    // MasterIntakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    // MasterIntakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);
    // MasterIntakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    // MasterIntakeMotor.setIdleMode(IdleMode.kCoast);

    MasterIntakeMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // System.out.println("Intake Sensor:" + (intakeSensor.get() ==
    // IntakeConstants.kIntakeSensorNoteDetected));
    // System.out.println("Output Sensor:" + (outputSensor.get() ==
    // IntakeConstants.kOutputSensorNoteDetected));
    if ((intakeSensor.get() == IntakeConstants.kIntakeSensorNoteDetected)
        || (outputSensor.get() == IntakeConstants.kOutputSensorNoteDetected))
      NoteDetectedLED.set(true);
    else NoteDetectedLED.set(false);

    if ((intakeSensor.get() == IntakeConstants.kIntakeSensorNoteDetected)
        && (outputSensor.get() != IntakeConstants.kOutputSensorNoteDetected))
      NoteReadyLED.set(true);
    else NoteReadyLED.set(false);

    intakeSpeed = SmartDashboard.getNumber("Intake Speed", intakeSpeed);
    indexSpeed = SmartDashboard.getNumber("Index Speed", indexSpeed);
    indexReverseSpeed = SmartDashboard.getNumber("Index Reverse Speed", indexReverseSpeed);

    // if ((intakeSensor.get() == IntakeConstants.kIntakeSensorNoteDetected)) {
    //   noteInShooter = true;
    // } else {
    //   noteInShooter = false;
    // }

    if ((intakeSensor.get() == IntakeConstants.kIntakeSensorNoteDetected)
        && (outputSensor.get() != IntakeConstants.kIntakeSensorNoteDetected)) {
      noteInShooter = true;
    } else {
      noteInShooter = false;
    }

    SmartDashboard.putBoolean("Note in the sensor", noteInShooter);
  }

  /** Starts motor intake but stops if a note is detected inside */

  /** Starts motor intake */
  public void StartIntake() {
    MasterIntakeMotor.set(intakeSpeed);
    speedOfIntake.append("The Intake Speed...");
  }

  public void StartIndex() {
    MasterIntakeMotor.set(indexSpeed);
    speedOfIntake.append("The Index Speed...");
  }

  public void StartIndexSlow() {
    MasterIntakeMotor.set(IntakeConstants.kIndexSpeedSlow);
    speedOfIntake.append("The Index Speed...");
  }

  public void StartReverseIndex() {
    MasterIntakeMotor.set(indexReverseSpeed);
    speedOfIntake.append("The reverse Index Speed...");
  }

  /** Stops motor intake */
  public void StopIntake() {
    MasterIntakeMotor.set(0);
    System.out.println("Intake stopped!");
  }
}
