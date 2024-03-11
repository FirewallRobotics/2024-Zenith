// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.AutonomousTrajectories;
import frc.robot.subsystems.AxleSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UltrasonicSensor;
import frc.robot.subsystems.VisionSubsystem;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();
  private final AxleSubsystem m_axle = new AxleSubsystem();
  private final VisionSubsystem m_vision = new VisionSubsystem();
  private final AutoAimSubsystem m_autoAim = new AutoAimSubsystem();
  private final UltrasonicSensor m_UltrasonicSensor = new UltrasonicSensor();
  private final LEDSubsystem m_LED = new LEDSubsystem();

  private final AutonomousTrajectories m_trajectories = new AutonomousTrajectories();

  // The driver's controller

  // Static for trigger
  static XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    SmartDashboard.putNumber("Shoot Speaker Speed", Constants.ShooterConstants.kShootSpeakerSpeed);
    SmartDashboard.putNumber("Shoot Amp Speed", Constants.ShooterConstants.kShootAmpSpeed);
    SmartDashboard.putNumber("Index Speed", Constants.IntakeConstants.kIndexSpeed);
    SmartDashboard.putNumber("Reverse Index Speed", Constants.IntakeConstants.kIndexReverseSpeed);
    SmartDashboard.putNumber("Intake Speed", Constants.IntakeConstants.kIntakeMotorSpeed);
    SmartDashboard.putNumber("Arm Speed", Constants.AxleConstants.kAxleTestSpeed);
    SmartDashboard.putNumber("Arm Test Angle", Constants.AxleConstants.kTestHeight);

    m_chooser.setDefaultOption(
        "Default Auto - Drive Straight 2 Meters", m_trajectories.getDriveStraight(m_robotDrive));
    // m_chooser.addOption(
    //     "Basic Auto: Start in front of Subwoofer, Score 2, Pick up middlfe note",
    //     m_trajectories.getScore2InFrontOfSubwooferCommand(
    //         m_robotDrive, m_axle, m_intake, m_shooter, m_climb, m_LED));
    m_chooser.addOption(
        "Basic Auto: Start in front of Subwoofer, Score 1, Pick up middlfe note",
        m_trajectories.getScore1InFrontOfSubwooferCommand(
            m_robotDrive, m_axle, m_intake, m_shooter, m_climb, m_LED));
    m_chooser.addOption(
        "Red: Start Right, Score 2 Speaker, Pick Up Right Note, Park Far Right",
        m_trajectories.getRedRightGrab1Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Red: Start Right, Score 3 Speaker, Pick Up Right + Middle Notes, Park Right of Stage",
        m_trajectories.getRedRightGrab2Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Red: Start Right, Score 4 Speaker, Pick Up All Notes, Park Right of Stage",
        m_trajectories.getRedRightGrab3Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Red: Start Left, Score 2 Speaker, Pick Up Left Note, Park Left of Stage",
        m_trajectories.getRedMiddleGrab1Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Red: Start Left, Score 3 Speaker, Pick Up Left + Middle Notes, Park Right of Stage",
        m_trajectories.getRedMiddleGrab2Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Red: Start Left, Score 4 Speaker, Pick Up All Notes, Park Far Right",
        m_trajectories.getRedMiddleGrab3Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Blue: Start Left, Score 2 Speaker, Pick Up Left Note, Park Far Left",
        m_trajectories.getBlueLeftGrab1Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Blue: Start Left, Score 3 Speaker, Pick Up Left + Middle Notes, Park Left of Stage",
        m_trajectories.getBlueLeftGrab2Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Blue: Start Left, Score 4 Speaker, Pick Up All Notes, Park Left of Stage",
        m_trajectories.getBlueLeftGrab3Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Blue: Start Right, Score 2 Speaker, Pick Up Right Note, Park Right of Stage",
        m_trajectories.getBlueMiddleGrab1Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Blue: Start Right, Score 3 Speaker, Pick Up Right + Middle Notes, Park Left of Stage",
        m_trajectories.getBlueMiddleGrab2Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));
    m_chooser.addOption(
        "Blue: Start Right, Score 4 Speaker, Pick Up All Notes, Park Far Left",
        m_trajectories.getBlueMiddleGrab3Note(
            m_robotDrive, m_autoAim, m_vision, m_axle, m_intake, m_LED, m_climb, m_shooter));

    SmartDashboard.putData(m_chooser);

    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband)
                    /*- (OIConstants.kDriveDeadband  * changeSpeed())*/ ,
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband)
                    /*  * changeSpeed() */ ,
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband)
                    /** changeSpeed() */
                    ,
                    true,
                    true),
            m_robotDrive));

    // m_LED.setDefaultCommand(new SetLEDOrange(m_LED));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(
            new ParallelCommandGroup(
                new SequentialCommandGroup(new ShootSpeakerCommand(m_shooter, m_axle, m_intake)),
                new SequentialCommandGroup(new WaitCommand(1), new IndexCommand(m_intake))));

    // new JoystickButton(m_driverController, Button.kRightBumper.value)
    //     .whileTrue(new ShootSpeakerCommand(m_shooter, m_axle, m_intake));

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(new IntakeFloorCommand(m_intake, m_axle, m_LED));

    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new AimSpeakerCommand(m_robotDrive, m_autoAim, m_vision, m_axle, m_LED));

    new JoystickButton(m_driverController, Button.kY.value).whileTrue(new AimAmpCommand(m_axle));

    new POVButton(m_driverController, 0).whileTrue(new ClimberUpCommand(m_climb, m_axle));

    new POVButton(m_driverController, 180).whileTrue(new ClimbDefaultCommand(m_climb, m_axle));

    // new JoystickButton(m_driverController, Button.kB.value).whileTrue(new AxleUpCommand(m_axle));
    // .whileFalse(new DefaultAxleHeightCommand(m_axle));
    // new JoystickButton(m_driverController, Button.kA.value)
    //     .whileTrue(new AxleEncoderTestCommand(m_axle));

    new JoystickButton(m_driverController, Button.kB.value).whileTrue(new AxleUpCommand(m_axle));

    new JoystickButton(m_driverController, Button.kA.value).whileTrue(new AxleDownCommand(m_axle));

    new POVButton(m_driverController, 270) // left D-pad
        .onTrue(new GyroSetZeroCommand(m_robotDrive));

    new POVButton(m_driverController, 90) // right D-pad
        .whileTrue(
            new ParallelCommandGroup(
                new ShootAmpCommand(m_shooter, m_axle, m_intake), new IndexCommand(m_intake)));
    // right D-pad for AMP shoot

    // new JoystickButton(m_driverController, Axis.kRightTrigger);

    // new JoystickButton(m_driverController, Button.kL2.value)
    //     .whileTrue(new ClimbMiddleCommand(m_climb));

    // new POVButton(m_driverController, 270).whileTrue(new ShootTrapCommand(m_shooter, m_axle));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    boolean tryTest = false;

    if (tryTest) {
      return m_trajectories.getDriveStraight(m_robotDrive);
    }

    // If true, you will perform one of the sequentual command groups rather than example
    // trajectories
    boolean doScoringAuto = true;

    if (doScoringAuto) {
      return m_chooser.getSelected();
    }

    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory myTrajectory;
    Trajectory bonusTrajectory = null;

    String chooser = "Example";

    if (chooser.equals("Example")) {
      myTrajectory = getExampleTrajectory(config);
    } else if (chooser.equals("Forward")) {
      myTrajectory = getForwardTrajectory(config);
    } else if (chooser.equals("Forward180")) {
      myTrajectory = getForward180Trajectory(config);
    } else if (chooser.equals("Diagonal")) {
      myTrajectory = getDiagonalTrajectory(config);
    } else if (chooser.equals("Diagonal90")) {
      myTrajectory = getDiagonal90Trajectory(config);
    } else if (chooser.equals("Curve")) {
      myTrajectory = getCurveTrajectory(config);
    } else if (chooser.equals("Curve180")) {
      myTrajectory = getCurve180Trajectory(config);
    } else if (chooser.equals("BigCurve180")) {
      myTrajectory = getBigCurve180Trajectory(config);
    } else if (chooser.equals("ForwardDown")) {
      myTrajectory = getForwardTrajectory(config);
      bonusTrajectory = getRightTrajectory(config);
    } else if (chooser.equals("Spin")) {
      myTrajectory = getSpinTrajectory(config);
    } else {
      myTrajectory = getNullTrajectory(config);
    }

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

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

    SwerveControllerCommand bonusSwerveControllerCommand = null;

    if (bonusTrajectory != null) {
      bonusSwerveControllerCommand =
          new SwerveControllerCommand(
              bonusTrajectory,
              m_robotDrive::getPose, // Functional interface to feed supplier
              DriveConstants.kDriveKinematics,

              // Position controllers
              new PIDController(AutoConstants.kPXController, 0, 0),
              new PIDController(AutoConstants.kPYController, 0, 0),
              thetaController,
              m_robotDrive::setModuleStates,
              m_robotDrive);
    }

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(myTrajectory.getInitialPose());

    if (bonusSwerveControllerCommand == null) {
      // Run path following command, then stop at the end.
      return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    } else {
      // Run path following commands, then stop at the end.
      return swerveControllerCommand
          .andThen(bonusSwerveControllerCommand)
          .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    }
  }

  private Trajectory getExampleTrajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    return exampleTrajectory;
  }

  private Trajectory getForwardTrajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory forwardTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 1 meter straight ahead of where we started, facing forward
            new Pose2d(1, 0, new Rotation2d(0)),
            config);

    return forwardTrajectory;
  }

  private Trajectory getForward180Trajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory forward180Trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 1 meter straight ahead of where we started, facing backwards
            new Pose2d(1, 0, new Rotation2d(Math.PI)),
            config);

    return forward180Trajectory;
  }

  private Trajectory getDiagonalTrajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory diagonalTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 1 meter straight ahead and 1 meter left? from where we started, facing forward
            new Pose2d(1, 1, new Rotation2d(0)),
            config);

    return diagonalTrajectory;
  }

  private Trajectory getDiagonal90Trajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory diagonal180Trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 1 meter straight ahead and 1 meter left? from where we started, facing left?
            new Pose2d(1, 1, new Rotation2d(-Math.PI / 2)),
            config);

    return diagonal180Trajectory;
  }

  private Trajectory getCurveTrajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory curveTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through this interior waypoint, making "halfpipe" curve path
            List.of(new Translation2d(0.2, -0.8)),
            // End 1 meter straight ahead and 1 meter right? of where we started, facing forward
            new Pose2d(1, -1, new Rotation2d(0)),
            config);

    return curveTrajectory;
  }

  private Trajectory getCurve180Trajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory curve180Trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through this interior waypoint, making "halfpipe" curve path
            List.of(new Translation2d(0.2, -0.8)),
            // End 1 meter straight ahead and 1 meter right? of where we started, facing backwards
            new Pose2d(1, -1, new Rotation2d(Math.PI)),
            config);

    return curve180Trajectory;
  }

  private Trajectory getBigCurve180Trajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory bigCurve180Trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through this interior waypoint, making an 'c' curve path to the left? of the
            // start
            List.of(new Translation2d(1, 1)),
            // End 2 meters straight ahead of where we started, facing backwards
            new Pose2d(2, 0, new Rotation2d(0)),
            config);

    return bigCurve180Trajectory;
  }

  private Trajectory getRightTrajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory downTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // End 1 meter straight right? of where we started, facing forward
            new Pose2d(0, -1, new Rotation2d(0)),
            config);

    return downTrajectory;
  }

  private Trajectory getSpinTrajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory spinTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // Rotate 360 degrees and end facing forward again
            new Pose2d(0, 0, new Rotation2d(2 * Math.PI)),
            config);

    return spinTrajectory;
  }

  private Trajectory getNullTrajectory(TrajectoryConfig config) {
    // An example trajectory to follow. All units in meters.
    Trajectory nullTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No additional interior waypoints
            List.of(),
            // Don't move from original starting point
            new Pose2d(0, 0, new Rotation2d(0)),
            config);

    return nullTrajectory;
  }

  private double changeSpeed() {
    // As of this moment, no one knows if up is - or +, this might need to change.
    if (m_UltrasonicSensor.inRange30() && m_driverController.getLeftY() > 0) {
      return m_UltrasonicSensor.speedNeeded();
    } else {
      return 0;
    }
  }

  static boolean RightInthershold() {
    double rightTriggerValue = m_driverController.getRightTriggerAxis();

    if (rightTriggerValue >= 0.5) {
      return true;
    }

    return false;
  }

  static boolean LeftTriggerInThershold() {
    double leftTriggerValue = m_driverController.getLeftTriggerAxis();

    if (leftTriggerValue >= 0.5) {
      return true;
    }
    return false;
  }
}
