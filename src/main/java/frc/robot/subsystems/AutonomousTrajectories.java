// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoAimSpeakerCommand;
import frc.robot.commands.AutoShootSpeakerCommand;
import frc.robot.commands.IntakeAxleHeightCommand;
import frc.robot.commands.IntakeFloorCommand;
import java.util.List;

public class AutonomousTrajectories extends SubsystemBase {
  /** Creates a new AutonomousTrajectories. */
  public AutonomousTrajectories() {
    trajectoryConfig =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private TrajectoryConfig trajectoryConfig;
  private ProfiledPIDController thetaController;

  /**
   * Starting from the left most starting position, it goes from start to a spot inbetween the
   * speaker and the first note
   */
  public Trajectory getRedRightMostTrajectory1(TrajectoryConfig config) {
    Trajectory rightMostTrajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No interior points
            List.of(),
            // End 1.25 meters straight ahead and 0.5 meters left of where we started, facing 30
            // degrees right
            new Pose2d(1.25, 0.5, new Rotation2d(Math.toRadians(-30))),
            config);

    return rightMostTrajectory1;
  }

  public Trajectory getRedRightMostTrajectory2FieldRelative(TrajectoryConfig config) {
    Trajectory rightMostTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 0.5 meters left of where we started, facing
            // 30 degrees right
            new Pose2d(1.25, 0.5, new Rotation2d(Math.toRadians(-30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, 1.0)),
            // End 1.25 meters straight ahead and 1.5 meters left of where we started, facing the +X
            // direction
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return rightMostTrajectory2;
  }

  public Trajectory getRedRightMostTrajectory3FieldRelative(TrajectoryConfig config) {
    Trajectory rightMostTrajectory3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 1.5 meters left of where we started, facing
            // the +X direction
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, 2.125)),
            // End 1.25 meters straight ahead and 0.5 meters left of where we started, facing the +X
            // direction
            new Pose2d(1.25, 2.75, new Rotation2d(Math.toRadians(30))),
            config);

    return rightMostTrajectory3;
  }

  /**
   * Starting from the left most starting position, it goes from start to a spot inbetween the
   * speaker and the first note
   */
  public Trajectory getBlueLeftMostTrajectory1(TrajectoryConfig config) {
    Trajectory leftMostTrajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Interior point to make a curve
            List.of(),
            // End 1.25 meters straight ahead and 0.5 meters right of where we started, facing 30
            // degrees left
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            config);

    return leftMostTrajectory1;
  }

  public Trajectory getBlueLeftMostTrajectory2FieldRelative(TrajectoryConfig config) {
    Trajectory leftMostTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 0.5 meters right of where we started, facing
            // 30 degrees left
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, -1.0)),
            // End 1.25 meters straight ahead and 1.5 meters right of where we started, facing the
            // +X direction
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return leftMostTrajectory2;
  }

  public Trajectory getBlueLeftMostTrajectory3FieldRelative(TrajectoryConfig config) {
    Trajectory leftMostTrajectory3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 1.5 meters right of where we started, facing
            // the +X direction
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, -2.125)),
            // End 1.25 meters straight ahead and 0.5 meters right of where we started, facing 30
            // degrees
            new Pose2d(1.25, -2.75, new Rotation2d(Math.toRadians(-30))),
            config);

    return leftMostTrajectory3;
  }

  /**
   * Starting from the middle starting position, it goes from start to a spot inbetween the speaker
   * and the first note
   */
  public Trajectory getRedMiddleTrajectory1(TrajectoryConfig config) {
    Trajectory middleTrajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Interior point to make a curve
            List.of(),
            // End 1.25 meters straight ahead and 0.5 meters right of where we started, facing 30
            // degrees left
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            config);

    return middleTrajectory1;
  }

  public Trajectory getRedMiddleTrajectory2FieldRelative(TrajectoryConfig config) {
    Trajectory middleTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 0.5 meters right of where we started, facing
            // 30 degrees left
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, -1.0)),
            // End 1.25 meters straight ahead and 1.5 meters right of where we started, facing the
            // +X direction
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return middleTrajectory2;
  }

  public Trajectory getRedMiddleTrajectory3FieldRelative(TrajectoryConfig config) {
    Trajectory middleTrajectory3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 1.5 meters right of where we started, facing
            // the +X direction
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, -2.125)),
            // End 1.25 meters straight ahead and 0.5 meters right of where we started, facing 30
            // degrees
            new Pose2d(1.25, -2.75, new Rotation2d(Math.toRadians(-30))),
            config);

    return middleTrajectory3;
  }

  /**
   * Starting from the middle starting position, it goes from start to a spot inbetween the speaker
   * and the first note
   */
  public Trajectory getBlueMiddleTrajectory1(TrajectoryConfig config) {
    Trajectory middleTrajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No interior points
            List.of(),
            // End 1.25 meters straight ahead and 0.5 meters left of where we started, facing 30
            // degrees right
            new Pose2d(1.25, 0.5, new Rotation2d(Math.toRadians(-30))),
            config);

    return middleTrajectory1;
  }

  public Trajectory getBlueMiddleTrajectory2FieldRelative(TrajectoryConfig config) {
    Trajectory middleTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 0.5 meters left of where we started, facing
            // 30 degrees right
            new Pose2d(1.25, 0.5, new Rotation2d(Math.toRadians(-30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, 1.0)),
            // End 1.25 meters straight ahead and 1.5 meters left of where we started, facing the +X
            // direction
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return middleTrajectory2;
  }

  public Trajectory getBlueMiddleTrajectory3FieldRelative(TrajectoryConfig config) {
    Trajectory middleTrajectory3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 1.5 meters left of where we started, facing
            // the +X direction
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, 2.125)),
            // End 1.25 meters straight ahead and 0.5 meters left of where we started, facing the +X
            // direction
            new Pose2d(1.25, 2.75, new Rotation2d(Math.toRadians(30))),
            config);

    return middleTrajectory3;
  }

  /**
   * Starting from the middle starting position, it goes from start to a spot inbetween the speaker
   * and the middle note
   */
  public Trajectory getRedMiddleAlternateTrajectory(TrajectoryConfig config) {
    Trajectory middleAlternateTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, -0.5)),
            // End 1.25 meters straight ahead and 1.5 meters right of where we started, facing the
            // +X direction
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return middleAlternateTrajectory;
  }

  /**
   * Starting from the middle starting position, it goes from start to a spot inbetween the speaker
   * and the middle note
   */
  public Trajectory getBlueMiddleAlternateTrajectory(TrajectoryConfig config) {
    Trajectory middleAlternateTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, 0.5)),
            // End 1.25 meters straight ahead and 1.5 meters left of where we started, facing the +X
            // direction
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return middleAlternateTrajectory;
  }

  /** Goes straight forward 2 meters for a standard mobility bonus and nothing else */
  public Trajectory getForwardTrajectory(TrajectoryConfig config) {
    Trajectory forwardTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 2 meters straight ahead of where we started, facing forward
            new Pose2d(2, 0, new Rotation2d(0)),
            config);

    return forwardTrajectory;
  }

  public Trajectory getRedParkAfterLeftNoteLeftPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.75, 1.0)),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(1.75, 0.75, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getRedParkAfterMiddleNoteLeftPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(2.25, -1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getRedParkAfterRightNoteLeftPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, -2.75, new Rotation2d(Math.toRadians(-30))),
            // Interior point to make a curve
            List.of(),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(3.25, -3.25, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getRedParkAfterLeftNoteRightPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, 2.75, new Rotation2d(Math.toRadians(30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.5, 2.25)),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(2.25, 2.0, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getRedParkAfterMiddleNoteRightPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(new Translation2d(1.5, 2.25)),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(2.25, 1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getRedParkAfterRightNoteRightPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, 0.5, new Rotation2d(Math.toRadians(-30))),
            // Interior point to make a curve
            List.of(),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(3.25, 0.0, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getBlueParkAfterRightNoteRightPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, 0.5, new Rotation2d(Math.toRadians(-30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.75, -1.0)),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(1.75, -0.75, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getBlueParkAfterMiddleNoteRightPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(2.25, 1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getBlueParkAfterLeftNoteRightPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, 2.75, new Rotation2d(Math.toRadians(30))),
            // Interior point to make a curve
            List.of(),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(3.25, 3.25, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getBlueParkAfterRightNoteLeftPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, -2.75, new Rotation2d(Math.toRadians(-30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.5, -2.25)),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(2.25, -2.0, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getBlueParkAfterMiddleNoteLeftPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(new Translation2d(1.5, -2.25)),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(2.25, -1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getBlueParkAfterLeftNoteLeftPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            // Interior point to make a curve
            List.of(),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            new Pose2d(3.25, 0.0, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public SwerveControllerCommand getTrajectoryCommand(
      Trajectory myTrajectory, DriveSubsystem m_robotDrive, ProfiledPIDController thetaController) {

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            myTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    return swerveControllerCommand;
  }

  private Command getRedRightGrab1Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getRedRightMostTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedParkAfterRightNoteRightPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_RedRightGrab2Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getRedRightMostTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedRightMostTrajectory2FieldRelative(trajectoryConfig),
            m_robotDrive,
            thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedParkAfterMiddleNoteRightPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_RedRightGrab3Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getRedRightMostTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedRightMostTrajectory2FieldRelative(trajectoryConfig),
            m_robotDrive,
            thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedRightMostTrajectory3FieldRelative(trajectoryConfig),
            m_robotDrive,
            thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedParkAfterLeftNoteRightPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_RedMiddleGrab1Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getRedMiddleTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedParkAfterLeftNoteLeftPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_RedMiddleGrab2Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getRedMiddleTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedMiddleTrajectory2FieldRelative(trajectoryConfig), m_robotDrive, thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedParkAfterMiddleNoteLeftPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_RedMiddleGrab3Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getRedMiddleTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedMiddleTrajectory2FieldRelative(trajectoryConfig), m_robotDrive, thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedMiddleTrajectory3FieldRelative(trajectoryConfig), m_robotDrive, thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getRedParkAfterRightNoteLeftPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_BlueLeftGrab1Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getBlueLeftMostTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueParkAfterLeftNoteLeftPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_BlueLeftGrab2Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getBlueLeftMostTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueLeftMostTrajectory2FieldRelative(trajectoryConfig),
            m_robotDrive,
            thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueParkAfterMiddleNoteLeftPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_BlueLeftGrab3Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getBlueLeftMostTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueLeftMostTrajectory2FieldRelative(trajectoryConfig),
            m_robotDrive,
            thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueLeftMostTrajectory3FieldRelative(trajectoryConfig),
            m_robotDrive,
            thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueParkAfterRightNoteLeftPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_BlueMiddleGrab1Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getBlueMiddleTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueParkAfterRightNoteRightPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_BlueMiddleGrab2Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getBlueMiddleTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueMiddleTrajectory2FieldRelative(trajectoryConfig), m_robotDrive, thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueParkAfterMiddleNoteRightPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_BlueMiddleGrab3Note(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED) {
    return new SequentialCommandGroup(
        getTrajectoryCommand(
            getBlueMiddleTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        /// new GoToRedNote1Command (or something like that),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueMiddleTrajectory2FieldRelative(trajectoryConfig), m_robotDrive, thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueMiddleTrajectory3FieldRelative(trajectoryConfig), m_robotDrive, thetaController),
        new IntakeAxleHeightCommand(),
        new IntakeFloorCommand(m_intake, m_axle, m_LED),
        new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
        new AutoShootSpeakerCommand(),
        getTrajectoryCommand(
            getBlueParkAfterLeftNoteRightPathTrajectory(trajectoryConfig),
            m_robotDrive,
            thetaController));
  }

  private Command m_driveStraight(DriveSubsystem m_robotDrive) {
    return getTrajectoryCommand(
        getForwardTrajectory(trajectoryConfig), m_robotDrive, thetaController);
  }
}
