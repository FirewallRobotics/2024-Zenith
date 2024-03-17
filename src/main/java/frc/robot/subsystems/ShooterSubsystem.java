// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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

  public static RelativeEncoder ShooterEncoder;
  public double finalVelocity = ShooterConstants.kTestVelocity;

  private SparkPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  public ShooterSubsystem() {
    MasterShooterMotor =
        new CANSparkMax(ShooterConstants.kMasterShooterMotorPort, MotorType.kBrushless);
    MinionShooterMotor =
        new CANSparkMax(ShooterConstants.kMinionShooterMotorPort, MotorType.kBrushless);

    MasterShooterMotor.restoreFactoryDefaults();
    MinionShooterMotor.restoreFactoryDefaults();

    MasterShooterMotor.setSmartCurrentLimit(35);
    MinionShooterMotor.setSmartCurrentLimit(35);

    DataLog log = DataLogManager.getLog();
    shooterActive = new StringLogEntry(log, "Shooters Shoot");

    MinionShooterMotor.follow(MasterShooterMotor, false);

    ShooterEncoder = MasterShooterMotor.getEncoder();
    m_pidController = MasterShooterMotor.getPIDController();

    m_pidController.setFeedbackDevice(ShooterEncoder);
    // PID coefficients
    kP = 0.0;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.0;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain Shooter:", kP);
    SmartDashboard.putNumber("I Gain Shooter:", kI);
    SmartDashboard.putNumber("D Gain Shooter:", kD);
    SmartDashboard.putNumber("I Zone Shooter:", kIz);
    SmartDashboard.putNumber("Feed Forward Shooter:", kFF);
    SmartDashboard.putNumber("Max Output Shooter:", kMaxOutput);
    SmartDashboard.putNumber("Min Output Shooter:", kMinOutput);

    SmartDashboard.putNumber("Testing Velocity", finalVelocity);
    // MasterShooterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward,
    // true);
    // MasterShooterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse,
    // true);

    // MasterShooterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);
    // MasterShooterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    // MasterIntakeMotor.setIdleMode(IdleMode.kCoast);

    MasterShooterMotor.burnFlash();
    MinionShooterMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter 11 Current:", MasterShooterMotor.getOutputCurrent());
    SmartDashboard.putNumber("Shooter 12 Current:", MinionShooterMotor.getOutputCurrent());

    // This method will be called once per scheduler run

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain Shooter:", 0);
    double i = SmartDashboard.getNumber("I Gain Shooter:", 0);
    double d = SmartDashboard.getNumber("D Gain Shooter:", 0);
    double iz = SmartDashboard.getNumber("I Zone Shooter:", 0);
    double ff = SmartDashboard.getNumber("Feed Forward Shooter:", 0);
    double max = SmartDashboard.getNumber("Max Output Shooter:", 0);
    double min = SmartDashboard.getNumber("Min Output Shooter:", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != kP)) {
      m_pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      m_pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      m_pidController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      m_pidController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      m_pidController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      m_pidController.setOutputRange(min, max);
    }

    shootSpeakerSpeed = SmartDashboard.getNumber("Shoot Speaker Speed", shootSpeakerSpeed);
    shootAmpSpeed = SmartDashboard.getNumber("Shoot Amp Speed", shootAmpSpeed);

    finalVelocity = SmartDashboard.getNumber("Testing Velocity", finalVelocity);

    SmartDashboard.putNumber("Current Velocity:", ShooterEncoder.getVelocity());
  }

  public void ShootAmp() {
    MasterShooterMotor.set(shootAmpSpeed);

    System.out.println("Shooting at the Amp...");
    shooterActive.append("Shooting at log Amp...");
    VisionSubsystem.UnicornNotify("ShootingAmp");
  }

  public void ShootSpeakerWithPID() {
    m_pidController.setReference(finalVelocity, CANSparkMax.ControlType.kVelocity);

    VisionSubsystem.UnicornNotify("ShootingSpeaker");
    System.out.println("Shooting at the Speaker...");
    shooterActive.append("Shooting at log Speaker...");
  }

  public void ShootSpeaker() {
    MasterShooterMotor.set(shootSpeakerSpeed);

    VisionSubsystem.UnicornNotify("ShootingSpeaker");
    System.out.println("Shooting at the Speaker...");
    shooterActive.append("Shooting at log Speaker...");
  }

  public void ShootTrap() {
    MasterShooterMotor.set(shootAmpSpeed);
  }

  public void StopShoot() {
    MasterShooterMotor.set(0);

    VisionSubsystem.UnicornNotify("");
  }

  public void ReverseShooter() {
    MasterShooterMotor.set(ShooterConstants.kReverseIndexSpeed);
  }
}
