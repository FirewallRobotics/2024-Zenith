// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AxleConstants;

public class AxleSubsystem extends SubsystemBase {
  /** Creates a new AxleSubsystem. */
  public static CANSparkMax MasterAxleMotor;

  public static CANSparkMax MinionAxleMotor;
  public static AbsoluteEncoder AxleEncoder;
  DigitalInput topLimitSwitch = new DigitalInput(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(1);
  private SparkPIDController AxlePIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public AxleSubsystem() {

    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0.3;
    kMaxOutput = 1;
    kMinOutput = -1;
    MasterAxleMotor =
        new CANSparkMax(
            AxleConstants.kMasterAxleMotorPort,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    MinionAxleMotor =
        new CANSparkMax(
            AxleConstants.kMinionAxleMotorPort,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    MasterAxleMotor.restoreFactoryDefaults();
    MinionAxleMotor.restoreFactoryDefaults();

    MasterAxleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    MasterAxleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    MinionAxleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    MinionAxleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    MinionAxleMotor.follow(MasterAxleMotor);

    AxleEncoder = MasterAxleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    AxleEncoder.setInverted(false);

    AxlePIDController.setP(kP);
    AxlePIDController.setI(kI);
    AxlePIDController.setD(kD);
    AxlePIDController.setIZone(kIz);
    AxlePIDController.setFF(kFF);
    AxlePIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void setMotorSpeed(double speed) {
    if (speed > 0) {
      if (topLimitSwitch.get()) {
        // We are going up and top limit is tripped so stop
        MasterAxleMotor.set(0);
      } else {
        // We are going up but top limit is not tripped so go at commanded speed
        MasterAxleMotor.set(speed);
      }
    } else {
      if (bottomLimitSwitch.get()) {
        // We are going down and bottom limit is tripped so stop
        MasterAxleMotor.set(0);
      } else {
        // We are going down but bottom limit is not tripped so go at commanded speed
        MasterAxleMotor.set(speed);
      }
    }
  }

  public void GravityOffset(double kdefaultheight) {
    // position measured when arm is horizontal (with Pheonix Tuner)
    double currentPos = AxleEncoder.getPosition();
    double radians = currentPos - AxleConstants.kMeasuredPosHorizontal;
    double cosineScalar = java.lang.Math.cos(radians);
    AxlePIDController.setFF(kFF * cosineScalar);
    AxlePIDController.setReference(kdefaultheight, CANSparkMax.ControlType.kPosition);
  }

  public double getAngle() {
    double currentPos = AxleEncoder.getPosition();
    double radians = currentPos - AxleConstants.kMeasuredPosHorizontal;
    return radians;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // setMotorSpeed(joystick.getRawAxis(2));
  }

  public void SetAimHeight(double angle) {
    GravityOffset(angle);
  }

  public void SetDefaultHeight() {
    GravityOffset(AxleConstants.kDefaultHeight);
  }

  public void SetIntakeHeight() {
    GravityOffset(AxleConstants.kIntakeHeight);
  }

  // public void DefaultAngle() {}

  // public void AimAmpAngle() {}

  // public void AimSpeakerAngle() {}

  // public void AimTrapAngle() {}

  // public void IntakeFloorAngle() {}

  // public void IntakeSourceAngle() {}
}
