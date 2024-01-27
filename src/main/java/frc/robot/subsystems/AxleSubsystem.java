// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AxleConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.AbsoluteEncoder;

public class AxleSubsystem extends SubsystemBase {
  /** Creates a new AxleSubsystem. */
  public static CANSparkMax masterAxleMotor;
  public static CANSparkMax minionAxleMotor;
  public static AbsoluteEncoder axleEncoder;
  DigitalInput topLimitSwitch = new DigitalInput(0);
  DigitalInput bottomLimitSwitch = new DigitalInput(1);


  public AxleSubsystem() {

    masterAxleMotor = new CANSparkMax(AxleConstants.kMasterAxleMotorPort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    minionAxleMotor = new CANSparkMax(AxleConstants.kMinionAxleMotorPort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    masterAxleMotor.restoreFactoryDefaults();
    minionAxleMotor.restoreFactoryDefaults();

    masterAxleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    masterAxleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    minionAxleMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    minionAxleMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    minionAxleMotor.follow(masterAxleMotor);

    axleEncoder = masterAxleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    axleEncoder.setInverted(false);
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
