// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.AxleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimSpeakerCommand extends Command {
  /** Creates a new ShootSpeakerCommand. */
  private final DriveSubsystem m_Drivetrain;

  private final AutoAimSubsystem m_AutoAim;
  private final VisionSubsystem m_Vision;
  private final AxleSubsystem m_Axle;

  private final LEDSubsystem m_LED;

  private boolean foundTargetAngle;
  private double targetAimAngle;

  private double targetTagXPosition;

  private boolean successfulAngleAim;
  private boolean successfulDriveAim;

  public AimSpeakerCommand(
      DriveSubsystem dt_Subsystem,
      AutoAimSubsystem aa_Subsystem,
      VisionSubsystem v_Subsystem,
      AxleSubsystem a_Subsystem,
      LEDSubsystem led_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Drivetrain = dt_Subsystem;
    m_AutoAim = aa_Subsystem;
    m_Vision = v_Subsystem;
    m_Axle = a_Subsystem;
    m_LED = led_Subsystem;

    addRequirements(dt_Subsystem);
    addRequirements(aa_Subsystem);
    addRequirements(v_Subsystem);
    addRequirements(a_Subsystem);
    addRequirements(led_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetAimAngle = 0.0;
    foundTargetAngle = false;

    targetTagXPosition = 0.0;

    successfulAngleAim = false;
    successfulDriveAim = false;
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    if (m_Vision.checkForSpeakerTag()) {

      float centerX = m_Vision.getSpeakerTagCenterX();
      float angleOfTag = m_Vision.getSpeakerTagRotationZ();

      float cameraDistanceToTag = m_Vision.getSpeakerTagDistanceToTag();
      double cameraDistanceToSpeaker =
          m_AutoAim.solveForDistanceToSpeaker(cameraDistanceToTag, angleOfTag);
      // NOTEWORTHY, may have to invert rotationZ to negative

      findAimAngle(cameraDistanceToSpeaker);
      findTagXPosition(cameraDistanceToSpeaker, angleOfTag);

      if (targetAimAngle > (AutoAimConstants.kMaxPhysicalAngleDegrees * Math.PI / 180)) {
        System.out.println("Too close");
        // Set LEDs for TOO CLOSE
      }

      successfulDriveAim =
          (Math.abs(targetTagXPosition - centerX) < VisionConstants.kDriveAimErrorRange);

      if (successfulDriveAim) {
        stopDrive();
      } else {
        // Turn robot so centerX approaches our targetTagXPosition

        if (centerX > targetTagXPosition) {
          rotateRightDrive();
        } else {
          rotateLeftDrive();
        }
      }
    } else {
      System.out.println("Target not found");
      // Set LEDs for NOT FOUND
    }

    if (foundTargetAngle) {
      m_Axle.SetAimHeight(targetAimAngle + AutoAimConstants.kPhysicalShooterAngleOffsetDegrees);

      successfulAngleAim =
          (Math.abs(
                  targetAimAngle
                      - (m_Axle.getAngle()
                          + (AutoAimConstants.kPhysicalShooterAngleOffsetDegrees * Math.PI / 180)))
              < (AutoAimConstants.kShooterAimErrorRangeDegrees * Math.PI / 180));

      System.out.println("Angle found - changing to angle");
      // Set LEDs for in range

      if (successfulAngleAim) {
        System.out.println("Successful angle aim");
      }
    } else {
      System.out.println("Not in range");
      // Set LEDs for not in range
    }

    if (successfulAngleAim && successfulDriveAim) {
      System.out.println("Successful Aim!");
      // Set LEDs for aimed successfully
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void findAimAngle(double cameraDistanceToSpeaker) {
    m_AutoAim.setLaunchDistance(
        cameraDistanceToSpeaker); // Converts from cam distance to launch distance
    targetAimAngle = m_AutoAim.SolveForAimAngle();

    if (targetAimAngle == -1) {
      foundTargetAngle = true;
    } else {
      foundTargetAngle = false;
    }
  }

  private void findTagXPosition(double cameraDistanceToSpeaker, double angleOfTag) {
    double launchDistanceToSpeaker =
        cameraDistanceToSpeaker + AutoAimConstants.kLaunchToCameraDifference;

    double driveAngle = m_AutoAim.solveForDriveAngle(launchDistanceToSpeaker, angleOfTag);
    double driveAngleDegrees = driveAngle * Math.PI / 180;

    double driveAnglePixels =
        driveAngleDegrees * VisionConstants.kCameraCenterX * 2 / VisionConstants.kCameraFOV;

    targetTagXPosition =
        VisionConstants.kCameraCenterX - driveAnglePixels; // May need to reverse - to a +
  }

  private void rotateLeftDrive() {
    m_Drivetrain.drive(0, 0, -AutoAimConstants.kDriveRotationPower, true, true);
  }

  private void rotateRightDrive() {
    m_Drivetrain.drive(0, 0, AutoAimConstants.kDriveRotationPower, true, true);
  }

  private void stopDrive() {
    m_Drivetrain.drive(0, 0, 0, true, true);
  }
}
