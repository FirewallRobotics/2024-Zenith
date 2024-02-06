// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public static CANSparkMax MasterIntakeMotor;

  public static CANSparkMax MinionIntakeMotor;
  public static AbsoluteEncoder ArmEncoder;

  public static DigitalInput intakeSensor;

  public static Subsystem ledSubsystem;

  public IntakeSubsystem() {
    MasterIntakeMotor =
        new CANSparkMax(IntakeConstants.kMasterIntakeMotorPort, MotorType.kBrushless);
    MinionIntakeMotor =
        new CANSparkMax(IntakeConstants.kMinionIntakeMotorPort, MotorType.kBrushless);
    intakeSensor = new DigitalInput(IntakeConstants.kIntakeSensorPort);
    final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
    {
    }
    ;
    MasterIntakeMotor.restoreFactoryDefaults();
    MinionIntakeMotor.restoreFactoryDefaults();

    MasterIntakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    MasterIntakeMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    MasterIntakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);
    MasterIntakeMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    // MasterIntakeMotor.setIdleMode(IdleMode.kCoast);

    MinionIntakeMotor.follow(MasterIntakeMotor, true);
  }

  @Override
  public void periodic() {}

  /** Starts motor intake but stops if a note is detected inside */
  public void sensorStartIntake() {
    if (intakeSensor.get() == IntakeConstants.kIntakeSensorNoteDetected) {
      StartIntake();
      ((LEDSubsystem) ledSubsystem).SetLights(150, 26, 192);
    } else {
      StopIntake();
      ((LEDSubsystem) ledSubsystem).SetLights(240, 79, 5);
    }
  }

  /** Starts motor intake */
  public void StartIntake() {
    MasterIntakeMotor.set(IntakeConstants.kIntakeMotorSpeed);
  }

  /** Stops motor intake */
  public void StopIntake() {
    MasterIntakeMotor.set(0);
  }
}
