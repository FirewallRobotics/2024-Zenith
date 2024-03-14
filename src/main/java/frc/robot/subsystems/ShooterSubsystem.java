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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  public static CANSparkMax MasterShooterMotor;

  public static CANSparkMax MinionShooterMotor;
  public static AbsoluteEncoder ArmEncoder;
  StringLogEntry shooterActive;

  private double shootSpeakerSpeed = Constants.ShooterConstants.kShootSpeakerSpeed;
  private double shootAmpSpeed = Constants.ShooterConstants.kShootAmpSpeed;

  public ShooterSubsystem() {
    MasterShooterMotor =
        new CANSparkMax(ShooterConstants.kMasterShooterMotorPort, MotorType.kBrushless);
    MinionShooterMotor =
        new CANSparkMax(ShooterConstants.kMinionShooterMotorPort, MotorType.kBrushless);

    MasterShooterMotor.restoreFactoryDefaults();
    MinionShooterMotor.restoreFactoryDefaults();
    DataLog log = DataLogManager.getLog();
    shooterActive = new StringLogEntry(log, "Shooters Shoot");

    // MasterShooterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
    // true);
    // MasterShooterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
    // true);

    // MasterShooterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);
    // MasterShooterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    // MasterIntakeMotor.setIdleMode(IdleMode.kCoast);

    MinionShooterMotor.follow(MasterShooterMotor, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    shootSpeakerSpeed = SmartDashboard.getNumber("Shoot Speaker Speed", shootSpeakerSpeed);
    shootAmpSpeed = SmartDashboard.getNumber("Shoot Amp Speed", shootAmpSpeed);
  }

  public void ShootAmp() {
    MasterShooterMotor.set(shootAmpSpeed);
    System.out.println("Shooting at the Amp...");
    shooterActive.append("Shooting at log Amp...");
    VisionSubsystem.UnicornNotify("ShootingAmp");
  }

  public void ShootSpeaker() {
    VisionSubsystem.UnicornNotify("ShootingSpeaker");
    MasterShooterMotor.set(shootSpeakerSpeed);
    System.out.println("Shooting at the Speaker...");
    shooterActive.append("Shooting at log Speaker...");
  }

  public void ShootTrap() {
    MasterShooterMotor.set(shootAmpSpeed);
  }

  public void StopShoot() {
    VisionSubsystem.UnicornNotify("");
    MasterShooterMotor.set(0);
  }

  public void ReverseShooter() {
    MasterShooterMotor.set(-shootAmpSpeed);
  }
}
