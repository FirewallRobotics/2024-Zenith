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

public class PositionAmpCommand extends Command {

  private final VisionSubsystem m_VisionSubsystem;
  private float ampTagCenterXStore;
  private float ampTagCenterYStore;

  private final DriveSubsystem m_DriveSubsystem;

  private double[] needPosForRing = VisionConstants.kCenterOfScreen;

  /** Creates a new PositionAmpCommand. */
  public PositionAmpCommand(VisionSubsystem v_Subsystem, DriveSubsystem dr_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_VisionSubsystem = v_Subsystem;
    m_DriveSubsystem = dr_Subsystem;

    addRequirements(v_Subsystem);
    addRequirements(dr_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampTagCenterXStore = m_VisionSubsystem.getAmpTagCenterX();
    ampTagCenterYStore = m_VisionSubsystem.getAmpTagCenterY();

    if (ampTagCenterXStore < needPosForRing[0]) {
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

    } else if (ampTagCenterXStore > needPosForRing[0]) {
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

    if (ampTagCenterYStore > needPosForRing[1]) {

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

    } else if (ampTagCenterYStore < needPosForRing[1]) {
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
  public void end(boolean interrupted) {
    if (interrupted == true) {
      m_DriveSubsystem.m_frontLeft.setDesiredState(
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      m_DriveSubsystem.m_frontRight.setDesiredState(
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      m_DriveSubsystem.m_rearLeft.setDesiredState(
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      m_DriveSubsystem.m_rearRight.setDesiredState(
          new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (ampTagCenterXStore == needPosForRing[0] && ampTagCenterYStore == needPosForRing[1]) {
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
}
