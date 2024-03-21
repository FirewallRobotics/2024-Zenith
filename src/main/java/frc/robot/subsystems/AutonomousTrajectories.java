// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoAimSpeakerCommand;
import frc.robot.commands.AutoBasicAimSpeakerCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeAxleHeightCommand;
import frc.robot.commands.IntakeFloorCommand;
import frc.robot.commands.ReverseShooterCommand;
import frc.robot.commands.ShootSpeakerCommand;
import java.util.List;

public class AutonomousTrajectories extends SubsystemBase {
  /** Creates a new AutonomousTrajectories. */
  private DriveSubsystem m_robotDrive;

  private ShooterSubsystem m_shooter;
  private IntakeSubsystem m_intake;
  private ClimbSubsystem m_climb;
  private AxleSubsystem m_axle;
  private VisionSubsystem m_vision;
  private AutoAimSubsystem m_autoAim;
  private LEDSubsystem m_LED;

  public AutonomousTrajectories(
      DriveSubsystem m_robotDrive,
      AutoAimSubsystem m_autoAim,
      VisionSubsystem m_vision,
      AxleSubsystem m_axle,
      IntakeSubsystem m_intake,
      LEDSubsystem m_LED,
      ClimbSubsystem m_climb,
      ShooterSubsystem m_shooter) {
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

    this.m_robotDrive = m_robotDrive;
    this.m_autoAim = m_autoAim;
    this.m_vision = m_vision;
    this.m_axle = m_axle;
    this.m_intake = m_intake;
    this.m_LED = m_LED;
    this.m_climb = m_climb;
    this.m_shooter = m_shooter;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private TrajectoryConfig trajectoryConfig;
  private ProfiledPIDController thetaController;

  // ALL VALUES IN METERS!!!

  // x distance from front of subwoofer to note
  private final double basicXDistanceToNote = 1.25;

  // x distance to initial shoot position (meant to be an inbetween point so that we don't push the
  // note before intaking it)
  private final double xDistanceToInitialShootPos = 0.75;

  // x distance from side positions (left or right of subwoofer) to note
  private final double xDistanceToNote = 1.25;

  // y distance from side positions to the ANGLED target position (we move slightly left or right of
  // the first note)
  private final double yStrafeForAngledNote = 0.5;

  // Angle in degrees to point towards speaker
  private final double angleTowardsSpeakerDegrees = 30;

  // x distance from note when curving away from notes (used in midpoints, i.e. List.of(1.25 - 0.25,
  // 1.0))
  private final double xCurveDistanceFromNote = 0.25;

  // y distance of the half way point between two notes
  private final double yCurveHalfwayBetweenNotes = 1.0;

  // y distance between two notes
  private final double yDistanceBetweenNotes = 1.4;

  //

  /** Starts from one side of the subwoofer directly facing the first note */
  public Trajectory getSideTrajectory1(TrajectoryConfig config, boolean rightOfSubwoofer) {
    int dirMultiplier = (rightOfSubwoofer) ? 1 : -1;

    Trajectory trajectory1 =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(
                xDistanceToInitialShootPos,
                yStrafeForAngledNote * dirMultiplier,
                new Rotation2d(Math.toRadians(angleTowardsSpeakerDegrees * -dirMultiplier))),
            config);

    return trajectory1;
  }

  /** Starts from one side of the subwoofer directly facing the first note */
  public Trajectory getSideTrajectory2(TrajectoryConfig config, boolean rightOfSubwoofer) {
    int dirMultiplier = (rightOfSubwoofer) ? 1 : -1;

    Trajectory trajectory1 =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(
                xDistanceToInitialShootPos,
                yStrafeForAngledNote * dirMultiplier,
                new Rotation2d(Math.toRadians(angleTowardsSpeakerDegrees * -dirMultiplier))),
            List.of(),
            new Pose2d(
                xDistanceToNote,
                yStrafeForAngledNote * dirMultiplier,
                new Rotation2d(Math.toRadians(angleTowardsSpeakerDegrees * -dirMultiplier))),
            config);

    return trajectory1;
  }

  public Trajectory getSideTrajectory3(TrajectoryConfig config, boolean rightOfSubwoofer) {
    int dirMultiplier = (rightOfSubwoofer) ? 1 : -1;

    Trajectory trajectory2 =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(
                xDistanceToNote,
                yStrafeForAngledNote * dirMultiplier,
                new Rotation2d(Math.toRadians(angleTowardsSpeakerDegrees * -dirMultiplier))),
            List.of(
                new Translation2d(
                    xCurveDistanceFromNote, yCurveHalfwayBetweenNotes * dirMultiplier)),
            new Pose2d(
                xDistanceToNote,
                yDistanceBetweenNotes * dirMultiplier,
                new Rotation2d(Math.toRadians(0))),
            config);

    return trajectory2;
  }

  public Trajectory getSideTrajectory4(TrajectoryConfig config, boolean rightOfSubwoofer) {
    int dirMultiplier = (rightOfSubwoofer) ? 1 : -1;

    Trajectory rightMostTrajectory3 =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(
                xDistanceToNote,
                yDistanceBetweenNotes * dirMultiplier,
                new Rotation2d(Math.toRadians(0))),
            List.of(
                new Translation2d(
                    xCurveDistanceFromNote,
                    (yDistanceBetweenNotes + yCurveHalfwayBetweenNotes) * dirMultiplier)),
            new Pose2d(
                xDistanceToNote,
                (yDistanceBetweenNotes * 2 - yStrafeForAngledNote) * dirMultiplier,
                new Rotation2d(Math.toRadians(angleTowardsSpeakerDegrees * dirMultiplier))),
            config);

    return rightMostTrajectory3;
  }

  public Trajectory getBasicAutoTrajectory(TrajectoryConfig config) {
    Trajectory forwardTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 2 meters straight ahead of where we started, facing forward
            new Pose2d(1.2, 0, new Rotation2d(0)),
            config);

    return forwardTrajectory;
  }

  public Trajectory getReverseBasicAutoTrajectory(TrajectoryConfig config) {
    Trajectory forwardTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(1.2, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 2 meters straight ahead of where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            config);

    return forwardTrajectory;
  }

  /**
   * Goes to one of the side notes from the front speaker position
   * @param config
   * @return Trajectory for this portion of auto
   */
  public Trajectory getBasicSideNoteAutoTrajectory(TrajectoryConfig config, boolean rightNote) {
    int dirMultiplier = (rightNote) ? -1 : 1;
    
    Trajectory curveTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 2 meters straight ahead of where we started, facing forward
            new Pose2d(xDistanceToNote, yDistanceBetweenNotes * dirMultiplier, new Rotation2d(0)),
            config);

    return curveTrajectory;
  }

  /**
   * Goes from one of the side notes to the front speaker position
   * @param config
   * @return Trajectory for this portion of auto
   */
  public Trajectory getBasicReverseSideNoteAutoTrajectory(TrajectoryConfig config, boolean rightNote) {
    int dirMultiplier = (rightNote) ? -1 : 1;
    
    Trajectory curveTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(xDistanceToNote, yDistanceBetweenNotes * dirMultiplier, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 2 meters straight ahead of where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            config);

    return curveTrajectory;
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

  public Trajectory getShortForwardTrajectory(TrajectoryConfig config) {
    Trajectory forwardTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 2 meters straight ahead of where we started, facing forward
            new Pose2d(1, 0, new Rotation2d(0)),
            config);

    return forwardTrajectory;
  }

  // In METERS
  private final double diagonalDistance = 1.5;

  public Trajectory getDiagonalTrajectory(
      TrajectoryConfig config, boolean rightOfSubwoofer, Pose2d currentPose) {
    int dirMultiplier = (rightOfSubwoofer) ? 1 : -1;

    Trajectory diagonalTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            currentPose,
            // No additional interior waypoints
            List.of(),
            // End 2 meters straight ahead of where we started, facing forward
            currentPose.transformBy(
                new Transform2d(
                    new Translation2d(
                        diagonalDistance * Math.cos(Math.toRadians(60)),
                        diagonalDistance * Math.sin(Math.toRadians(60)) * dirMultiplier),
                    new Rotation2d())),
            config);

    return diagonalTrajectory;
  }

  public Trajectory getRedParkAfterLeftNoteLeftPathTrajectory(TrajectoryConfig config) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            List.of(new Translation2d(1.75, 1.0)),
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

  public Trajectory getParkAfterAmpSideNoteTrajectory(TrajectoryConfig config, Pose2d currentPose) {
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            currentPose,
            // Interior point to make a curve
            List.of(),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X
            // direction
            currentPose.transformBy(new Transform2d(new Translation2d(), new Rotation2d())),
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
      Trajectory myTrajectory, ProfiledPIDController thetaController) {

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

  public Command getSideOfSubwooferRoutine(int notesScored, boolean rightOfSubwoofer) {
    Command autoRoutine =
        new SequentialCommandGroup(
            getTrajectoryCommand(
                getSideTrajectory1(trajectoryConfig, rightOfSubwoofer), thetaController),
            new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
            getShootCommandWithTimeout(m_shooter, m_axle, m_intake),
            getIntakeWithTrajectory(
                getTrajectoryCommand(
                    getSideTrajectory2(trajectoryConfig, rightOfSubwoofer), thetaController)),
            new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
            getShootCommandWithTimeout(m_shooter, m_axle, m_intake));

    // if (notesScored >= 3) {
    //   autoRoutine.andThen(
    //       getIntakeWithTrajectory(
    //           getTrajectoryCommand(
    //               getSideTrajectory3(trajectoryConfig, rightOfSubwoofer), thetaController)),
    //       new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
    //       getShootCommandWithTimeout(m_shooter, m_axle, m_intake));
    // }

    // if (notesScored >= 4) {
    //   autoRoutine.andThen(
    //       getIntakeWithTrajectory(
    //           getTrajectoryCommand(
    //               getSideTrajectory4(trajectoryConfig, rightOfSubwoofer), thetaController)),
    //       new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
    //       getShootCommandWithTimeout(m_shooter, m_axle, m_intake));
    // }

    return autoRoutine;
  }

  public Command getScore2FromSubwooferCommand() {
    return new SequentialCommandGroup(
        getTrajectoryCommand(getBasicAutoTrajectory(trajectoryConfig), thetaController),
        new AutoBasicAimSpeakerCommand(m_axle, m_climb).withTimeout(0.5),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),
        new IntakeAxleHeightCommand(m_axle, m_climb).withTimeout(0.5),
        new IntakeFloorCommand(m_intake, m_axle, m_LED).withTimeout(0.5),
        new AutoBasicAimSpeakerCommand(m_axle, m_climb).withTimeout(0.5),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake));
  }

  public Command getScore1InFrontOfSubwooferCommand() {
    return new SequentialCommandGroup(
        new AutoBasicAimSpeakerCommand(m_axle, m_climb),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),
        getTrajectoryCommand(getBasicAutoTrajectory(trajectoryConfig), thetaController));
  }

  public Command getScore2InFrontOfSubwooferCommand() {
    return new SequentialCommandGroup(
        new AutoBasicAimSpeakerCommand(m_axle, m_climb),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),
        new WaitCommand(1),
        getIntakeWithTrajectory(
            getTrajectoryCommand(getBasicAutoTrajectory(trajectoryConfig), thetaController)),
        getTrajectoryCommand(getReverseBasicAutoTrajectory(trajectoryConfig), thetaController),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake));
  }

  public Command getScore3InFrontOfSubwooferCommand(boolean secondNoteRight) {
    return new SequentialCommandGroup(
        new AutoBasicAimSpeakerCommand(m_axle, m_climb),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),

        new WaitCommand(1),
        getIntakeWithTrajectory(
            getTrajectoryCommand(getBasicAutoTrajectory(trajectoryConfig), thetaController)),
        getTrajectoryCommand(getReverseBasicAutoTrajectory(trajectoryConfig), thetaController),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),

        new WaitCommand(1),
        getIntakeWithTrajectory(
            getTrajectoryCommand(getBasicSideNoteAutoTrajectory(trajectoryConfig, secondNoteRight), thetaController)),
        getTrajectoryCommand(getBasicReverseSideNoteAutoTrajectory(trajectoryConfig, secondNoteRight), thetaController),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake));
  }

  public Command getScore4InFrontOfSubwooferCommand(boolean secondNoteRight) {
    return new SequentialCommandGroup(
        new AutoBasicAimSpeakerCommand(m_axle, m_climb),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),

        new WaitCommand(1),
        getIntakeWithTrajectory(
            getTrajectoryCommand(getBasicAutoTrajectory(trajectoryConfig), thetaController)),
        getTrajectoryCommand(getReverseBasicAutoTrajectory(trajectoryConfig), thetaController),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),

        new WaitCommand(1),
        getIntakeWithTrajectory(
            getTrajectoryCommand(getBasicSideNoteAutoTrajectory(trajectoryConfig, secondNoteRight), thetaController)),
        getTrajectoryCommand(getBasicReverseSideNoteAutoTrajectory(trajectoryConfig, secondNoteRight), thetaController),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),

        new WaitCommand(1),
        getIntakeWithTrajectory(
            getTrajectoryCommand(getBasicSideNoteAutoTrajectory(trajectoryConfig, !secondNoteRight), thetaController)),
        getTrajectoryCommand(getBasicReverseSideNoteAutoTrajectory(trajectoryConfig, !secondNoteRight), thetaController),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake));
  }

  public Command getRedScore1OnRightSideOfSubwooferCommand() {
    return new SequentialCommandGroup(
        new AutoBasicAimSpeakerCommand(m_axle, m_climb),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),
        getTrajectoryCommand(getShortForwardTrajectory(trajectoryConfig), thetaController),
        getTrajectoryCommand(
            getDiagonalTrajectory(trajectoryConfig, true, m_robotDrive.getPose()),
            thetaController));
  }

  public Command getRedScore1OnLeftSideOfSubwooferCommand() {
    return new SequentialCommandGroup(
        new AutoBasicAimSpeakerCommand(m_axle, m_climb),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),
        getTrajectoryCommand(getForwardTrajectory(trajectoryConfig), thetaController),
        getTrajectoryCommand(
            getDiagonalTrajectory(trajectoryConfig, false, m_robotDrive.getPose()),
            thetaController));
  }

  public Command getBlueScore1OnRightSideOfSubwooferCommand() {
    return new SequentialCommandGroup(
        new AutoBasicAimSpeakerCommand(m_axle, m_climb),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),
        getTrajectoryCommand(getForwardTrajectory(trajectoryConfig), thetaController),
        getTrajectoryCommand(
            getDiagonalTrajectory(trajectoryConfig, true, m_robotDrive.getPose()),
            thetaController));
  }

  public Command getBlueScore1OnLeftSideOfSubwooferCommand() {
    return new SequentialCommandGroup(
        new AutoBasicAimSpeakerCommand(m_axle, m_climb),
        getShootCommandWithTimeout(m_shooter, m_axle, m_intake),
        getTrajectoryCommand(getShortForwardTrajectory(trajectoryConfig), thetaController),
        getTrajectoryCommand(
            getDiagonalTrajectory(trajectoryConfig, true, m_robotDrive.getPose()),
            thetaController));
  }

  private final double shootDelay = 2.5;
  private final double shooterTimeout = shootDelay + 0.5;

  public Command getShootCommandWithTimeout(
      ShooterSubsystem m_shooter, AxleSubsystem m_axle, IntakeSubsystem m_intake) {
    return new ParallelCommandGroup(
            new SequentialCommandGroup(new ShootSpeakerCommand(m_shooter, m_axle, m_intake)),
            new SequentialCommandGroup(new WaitCommand(shootDelay), new IndexCommand(m_intake)))
        .withTimeout(shooterTimeout);
  }

  public Command getIntakeWithTrajectory(Command trajectoryCommand) {
    return new SequentialCommandGroup(
        new IntakeAxleHeightCommand(m_axle, m_climb).withTimeout(0.5),
        new ParallelCommandGroup(
            trajectoryCommand, new IntakeFloorCommand(m_intake, m_axle, m_LED)),
        new ReverseShooterCommand(m_shooter, m_intake, m_LED));
  }

  public Command getDriveStraight() {
    return getTrajectoryCommand(getForwardTrajectory(trajectoryConfig), thetaController);
  }

  //   public Command getBlueLeftGrab1Note(
  //       DriveSubsystem m_robotDrive,
  //       AutoAimSubsystem m_autoAim,
  //       VisionSubsystem m_vision,
  //       AxleSubsystem m_axle,
  //       IntakeSubsystem m_intake,
  //       LEDSubsystem m_LED,
  //       ClimbSubsystem m_climb,
  //       ShooterSubsystem m_shooter) {
  //     return new SequentialCommandGroup(
  //         getTrajectoryCommand(
  //             getBlueLeftMostTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         /// new GoToRedNote1Command (or something like that),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueParkAfterLeftNoteLeftPathTrajectory(trajectoryConfig),
  //             m_robotDrive,
  //             thetaController));
  //   }

  //   public Command getBlueLeftGrab2Note(
  //       DriveSubsystem m_robotDrive,
  //       AutoAimSubsystem m_autoAim,
  //       VisionSubsystem m_vision,
  //       AxleSubsystem m_axle,
  //       IntakeSubsystem m_intake,
  //       LEDSubsystem m_LED,
  //       ClimbSubsystem m_climb,
  //       ShooterSubsystem m_shooter) {
  //     return new SequentialCommandGroup(
  //         getTrajectoryCommand(
  //             getBlueLeftMostTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         /// new GoToRedNote1Command (or something like that),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueLeftMostTrajectory2FieldRelative(trajectoryConfig),
  //             m_robotDrive,
  //             thetaController),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueParkAfterMiddleNoteLeftPathTrajectory(trajectoryConfig),
  //             m_robotDrive,
  //             thetaController));
  //   }

  //   public Command getBlueLeftGrab3Note(
  //       DriveSubsystem m_robotDrive,
  //       AutoAimSubsystem m_autoAim,
  //       VisionSubsystem m_vision,
  //       AxleSubsystem m_axle,
  //       IntakeSubsystem m_intake,
  //       LEDSubsystem m_LED,
  //       ClimbSubsystem m_climb,
  //       ShooterSubsystem m_shooter) {
  //     return new SequentialCommandGroup(
  //         getTrajectoryCommand(
  //             getBlueLeftMostTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         /// new GoToRedNote1Command (or something like that),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueLeftMostTrajectory2FieldRelative(trajectoryConfig),
  //             m_robotDrive,
  //             thetaController),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueLeftMostTrajectory3FieldRelative(trajectoryConfig),
  //             m_robotDrive,
  //             thetaController),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueParkAfterRightNoteLeftPathTrajectory(trajectoryConfig),
  //             m_robotDrive,
  //             thetaController));
  //   }

  //   public Command getBlueMiddleGrab1Note(
  //       DriveSubsystem m_robotDrive,
  //       AutoAimSubsystem m_autoAim,
  //       VisionSubsystem m_vision,
  //       AxleSubsystem m_axle,
  //       IntakeSubsystem m_intake,
  //       LEDSubsystem m_LED,
  //       ClimbSubsystem m_climb,
  //       ShooterSubsystem m_shooter) {
  //     return new SequentialCommandGroup(
  //         getTrajectoryCommand(
  //             getBlueMiddleTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         /// new GoToRedNote1Command (or something like that),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueParkAfterRightNoteRightPathTrajectory(trajectoryConfig),
  //             m_robotDrive,
  //             thetaController));
  //   }

  //   public Command getBlueMiddleGrab2Note(
  //       DriveSubsystem m_robotDrive,
  //       AutoAimSubsystem m_autoAim,
  //       VisionSubsystem m_vision,
  //       AxleSubsystem m_axle,
  //       IntakeSubsystem m_intake,
  //       LEDSubsystem m_LED,
  //       ClimbSubsystem m_climb,
  //       ShooterSubsystem m_shooter) {
  //     return new SequentialCommandGroup(
  //         getTrajectoryCommand(
  //             getBlueMiddleTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         /// new GoToRedNote1Command (or something like that),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueMiddleTrajectory2FieldRelative(trajectoryConfig), m_robotDrive,
  // thetaController),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueParkAfterMiddleNoteRightPathTrajectory(trajectoryConfig),
  //             m_robotDrive,
  //             thetaController));
  //   }

  //   public Command getBlueMiddleGrab3Note(
  //       DriveSubsystem m_robotDrive,
  //       AutoAimSubsystem m_autoAim,
  //       VisionSubsystem m_vision,
  //       AxleSubsystem m_axle,
  //       IntakeSubsystem m_intake,
  //       LEDSubsystem m_LED,
  //       ClimbSubsystem m_climb,
  //       ShooterSubsystem m_shooter) {
  //     return new SequentialCommandGroup(
  //         getTrajectoryCommand(
  //             getBlueMiddleTrajectory1(trajectoryConfig), m_robotDrive, thetaController),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         /// new GoToRedNote1Command (or something like that),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueMiddleTrajectory2FieldRelative(trajectoryConfig), m_robotDrive,
  // thetaController),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueMiddleTrajectory3FieldRelative(trajectoryConfig), m_robotDrive,
  // thetaController),
  //         new IntakeAxleHeightCommand(m_axle, m_climb),
  //         new IntakeFloorCommand(m_intake, m_axle, m_LED),
  //         new AutoAimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle),
  //         new AutoShootSpeakerCommand(m_shooter, m_intake),
  //         getTrajectoryCommand(
  //             getBlueParkAfterLeftNoteRightPathTrajectory(trajectoryConfig),
  //             m_robotDrive,
  //             thetaController));
  //   }
}
