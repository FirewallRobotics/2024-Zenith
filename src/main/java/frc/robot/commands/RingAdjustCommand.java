// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RingAdjustCommand extends Command {

  private final VisionSubsystem m_VisionSubsystem;
  private final DriveSubsystem m_DriveSubsystem;
  private float xOfRingOnGrid;
  private float yOfRingOnGrid;
  private double[] needPosForRing = VisionConstants.kCenterOfScreen;
  private double[] rangeForXPosRing;
  private double[] rangeForYPosRing;

  /** Creates a new RingAdjustCommand. */
  public RingAdjustCommand(VisionSubsystem v_Subsystem, DriveSubsystem dr_Subsystem) {
    m_VisionSubsystem = v_Subsystem;
    m_DriveSubsystem = dr_Subsystem;

    /** Gets the range for X and Y. Index 0 is the minimum. Index 1 is the max. */
    rangeForXPosRing[0] = needPosForRing[0] - 100;
    rangeForXPosRing[1] = needPosForRing[0] + 100;

    rangeForYPosRing[0] = needPosForRing[1] - 100;
    rangeForYPosRing[1] = needPosForRing[1] + 100;

    addRequirements(v_Subsystem);
    addRequirements(dr_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xOfRingOnGrid = m_VisionSubsystem.getPixelX();
    yOfRingOnGrid = m_VisionSubsystem.getPixelY();

    if (xOfRingOnGrid > rangeForXPosRing[1]) {
      m_DriveSubsystem.m_frontLeft.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(-90)));
      m_DriveSubsystem.m_frontRight.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(-90)));
      m_DriveSubsystem.m_rearLeft.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(-90)));
      m_DriveSubsystem.m_rearRight.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(-90)));

    } else if (xOfRingOnGrid < rangeForYPosRing[0]) {
      m_DriveSubsystem.m_frontLeft.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(90)));
      m_DriveSubsystem.m_frontRight.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(90)));
      m_DriveSubsystem.m_rearLeft.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(90)));
      m_DriveSubsystem.m_rearRight.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(90)));
    }

    if (yOfRingOnGrid > rangeForYPosRing[1]) {

      m_DriveSubsystem.m_frontLeft.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(0)));
      m_DriveSubsystem.m_frontRight.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(0)));
      m_DriveSubsystem.m_rearLeft.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(0)));
      m_DriveSubsystem.m_rearRight.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(0)));

    } else if (yOfRingOnGrid < rangeForYPosRing[0]) {
      m_DriveSubsystem.m_frontLeft.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(180)));
      m_DriveSubsystem.m_frontRight.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(180)));
      m_DriveSubsystem.m_rearLeft.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(180)));
      m_DriveSubsystem.m_rearRight.setDesiredState(
          new SwerveModuleState(
              DriveConstants.kMaxSpeedMetersPerSecond, Rotation2d.fromDegrees(180)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (ringInXRange() && ringInYRange()) {
      m_DriveSubsystem.m_frontLeft.setDesiredState(
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      m_DriveSubsystem.m_frontRight.setDesiredState(
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      m_DriveSubsystem.m_rearLeft.setDesiredState(
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      m_DriveSubsystem.m_rearRight.setDesiredState(
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)));

      return true;
    }
    return false;
  }

  private boolean ringInXRange() {
    if (xOfRingOnGrid > rangeForXPosRing[0] && xOfRingOnGrid < rangeForXPosRing[1]) {
      return true;
    }
    return false;
  }

  private boolean ringInYRange() {
    if (yOfRingOnGrid > rangeForYPosRing[0] && yOfRingOnGrid < rangeForYPosRing[1]) {
      return true;
    }
    return false;
  }
}
