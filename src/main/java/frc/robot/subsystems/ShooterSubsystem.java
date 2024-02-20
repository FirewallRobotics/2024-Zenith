// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public static CANSparkMax MasterShooterMotor;

  public static CANSparkMax MinionShooterMotor;
  public static AbsoluteEncoder ArmEncoder;

  public ShooterSubsystem() {
    MasterShooterMotor =
        new CANSparkMax(ShooterConstants.kMasterShooterMotorPort, MotorType.kBrushless);
    MinionShooterMotor =
        new CANSparkMax(ShooterConstants.kMinionShooterMotorPort, MotorType.kBrushless);

    MasterShooterMotor.restoreFactoryDefaults();
    MinionShooterMotor.restoreFactoryDefaults();

    MasterShooterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    MasterShooterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    MasterShooterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);
    MasterShooterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    // MasterIntakeMotor.setIdleMode(IdleMode.kCoast);

    MinionShooterMotor.follow(MasterShooterMotor, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ShootAmp() {
    MasterShooterMotor.set(ShooterConstants.kShooterMotorSpeed);
  }

  public void ShootSpeaker() {
    MasterShooterMotor.set(ShooterConstants.kShooterMotorSpeed);
  }

  public void ShootTrap() {
    MasterShooterMotor.set(ShooterConstants.kShooterMotorSpeed);
  }

  public void StopShoot() {
    MasterShooterMotor.set(0);
  }
}
