// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AxleConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.AbsoluteEncoder;

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
    MasterAxleMotor = new CANSparkMax(AxleConstants.kMasterAxleMotorPort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    MinionAxleMotor = new CANSparkMax(AxleConstants.kMinionAxleMotorPort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

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

  public void GravityOffset(double kdefaultheight) {
  double kMeasuredPosHorizontal =
      .512; // position measured when arm is horizontal (with Pheonix Tuner)
  double currentPos = AxleEncoder.getPosition();
  double radians = currentPos - kMeasuredPosHorizontal;
  double cosineScalar = java.lang.Math.cos(radians);
  AxlePIDController.setFF(kFF * cosineScalar);
  AxlePIDController.setReference(kdefaultheight, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void DefaultAngle(){

  }

  public void AimAmpAngle(){

  }

  public void AimSpeakerAngle(){

  }

  public void AimTrapAngle(){

  }

  public void IntakeFloorAngle(){

  }

  public void IntakeSourceAngle(){
    
  }
}
