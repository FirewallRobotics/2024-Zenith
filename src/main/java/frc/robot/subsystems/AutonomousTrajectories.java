// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonomousTrajectories extends SubsystemBase {
  /** Creates a new AutonomousTrajectories. */
  public AutonomousTrajectories() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**Starting from the left most starting position, it goes from start to a spot inbetween the speaker and the first note */
  public Trajectory getRedRightMostTrajectory1(TrajectoryConfig config) {
    Trajectory rightMostTrajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No interior points
            List.of(),
            // End 1.25 meters straight ahead and 0.5 meters left of where we started, facing 30 degrees right
            new Pose2d(1.25, 0.5, new Rotation2d(Math.toRadians(-30))),
            config);

    return rightMostTrajectory1;
  }

  public Trajectory getRedRightMostTrajectory2FieldRelative(TrajectoryConfig config){
    Trajectory rightMostTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 0.5 meters left of where we started, facing 30 degrees right
            new Pose2d(1.25, 0.5, new Rotation2d(Math.toRadians(-30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, 1.0)),
            // End 1.25 meters straight ahead and 1.5 meters left of where we started, facing the +X direction
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return rightMostTrajectory2;
  }

  public Trajectory getRedRightMostTrajectory3FieldRelative(TrajectoryConfig config){
    Trajectory rightMostTrajectory3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 1.5 meters left of where we started, facing the +X direction
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, 2.125)),
            // End 1.25 meters straight ahead and 0.5 meters left of where we started, facing the +X direction
            new Pose2d(1.25, 2.75, new Rotation2d(Math.toRadians(30))),
            config);

    return rightMostTrajectory3;
  }

  /**Starting from the left most starting position, it goes from start to a spot inbetween the speaker and the first note */
  public Trajectory getBlueLeftMostTrajectory1(TrajectoryConfig config) {
    Trajectory leftMostTrajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Interior point to make a curve
            List.of(),
            // End 1.25 meters straight ahead and 0.5 meters right of where we started, facing 30 degrees left
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            config);

    return leftMostTrajectory1;
  }

  public Trajectory getBlueLeftMostTrajectory2FieldRelative(TrajectoryConfig config){
    Trajectory leftMostTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 0.5 meters right of where we started, facing 30 degrees left
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, -1.0)),
            // End 1.25 meters straight ahead and 1.5 meters right of where we started, facing the +X direction
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return leftMostTrajectory2;
  }

  public Trajectory getBlueLeftMostTrajectory3FieldRelative(TrajectoryConfig config){
    Trajectory leftMostTrajectory3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 1.5 meters right of where we started, facing the +X direction
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, -2.125)),
            // End 1.25 meters straight ahead and 0.5 meters right of where we started, facing 30 degrees 
            new Pose2d(1.25, -2.75, new Rotation2d(Math.toRadians(-30))),
            config);

    return leftMostTrajectory3;
  }

  /**Starting from the middle starting position, it goes from start to a spot inbetween the speaker and the first note */
  public Trajectory getRedMiddleTrajectory1(TrajectoryConfig config) {
    Trajectory middleTrajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Interior point to make a curve
            List.of(),
            // End 1.25 meters straight ahead and 0.5 meters right of where we started, facing 30 degrees left
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            config);

    return middleTrajectory1;
  }

  public Trajectory getRedMiddleTrajectory2FieldRelative(TrajectoryConfig config){
    Trajectory middleTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 0.5 meters right of where we started, facing 30 degrees left
            new Pose2d(1.25, -0.5, new Rotation2d(Math.toRadians(30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, -1.0)),
            // End 1.25 meters straight ahead and 1.5 meters right of where we started, facing the +X direction
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return middleTrajectory2;
  }

  public Trajectory getRedMiddleTrajectory3FieldRelative(TrajectoryConfig config){
    Trajectory middleTrajectory3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 1.5 meters right of where we started, facing the +X direction
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, -2.125)),
            // End 1.25 meters straight ahead and 0.5 meters right of where we started, facing 30 degrees 
            new Pose2d(1.25, -2.75, new Rotation2d(Math.toRadians(-30))),
            config);

    return middleTrajectory3;
  }

  /**Starting from the middle starting position, it goes from start to a spot inbetween the speaker and the first note */
  public Trajectory getBlueMiddleTrajectory1(TrajectoryConfig config) {
    Trajectory middleTrajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // No interior points
            List.of(),
            // End 1.25 meters straight ahead and 0.5 meters left of where we started, facing 30 degrees right
            new Pose2d(1.25, 0.5, new Rotation2d(Math.toRadians(-30))),
            config);

    return middleTrajectory1;
  }

  public Trajectory getBlueMiddleTrajectory2FieldRelative(TrajectoryConfig config){
    Trajectory middleTrajectory2 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 0.5 meters left of where we started, facing 30 degrees right
            new Pose2d(1.25, 0.5, new Rotation2d(Math.toRadians(-30))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, 1.0)),
            // End 1.25 meters straight ahead and 1.5 meters left of where we started, facing the +X direction
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return middleTrajectory2;
  }

  public Trajectory getBlueMiddleTrajectory3FieldRelative(TrajectoryConfig config){
    Trajectory middleTrajectory3 =
        TrajectoryGenerator.generateTrajectory(
            // Start at 1.25 meters straight ahead and 1.5 meters left of where we started, facing the +X direction
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, 2.125)),
            // End 1.25 meters straight ahead and 0.5 meters left of where we started, facing the +X direction
            new Pose2d(1.25, 2.75, new Rotation2d(Math.toRadians(30))),
            config);

    return middleTrajectory3;
  }

  /**Starting from the middle starting position, it goes from start to a spot inbetween the speaker and the middle note */
  public Trajectory getRedMiddleAlternateTrajectory(TrajectoryConfig config) {
    Trajectory middleAlternateTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, -0.5)),
            // End 1.25 meters straight ahead and 1.5 meters right of where we started, facing the +X direction
            new Pose2d(1.25, -1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return middleAlternateTrajectory;
  }

  /**Starting from the middle starting position, it goes from start to a spot inbetween the speaker and the middle note */
  public Trajectory getBlueMiddleAlternateTrajectory(TrajectoryConfig config) {
    Trajectory middleAlternateTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Interior point to make a curve
            List.of(new Translation2d(1.0, 0.5)),
            // End 1.25 meters straight ahead and 1.5 meters left of where we started, facing the +X direction
            new Pose2d(1.25, 1.5, new Rotation2d(Math.toRadians(0))),
            config);

    return middleAlternateTrajectory;
  }

  /**Goes straight forward 2 meters for a standard mobility bonus and nothing else */
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

  public Trajectory getRedParkAfterLeftNoteLeftPathTrajectory(TrajectoryConfig config){
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(30))),
            // Interior point to make a curve
            List.of(new Translation2d(0.75, -0.5)),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X direction
            new Pose2d(1, -0.75, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getRedParkAfterLeftNoteRightPathTrajectory(TrajectoryConfig config){
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees left
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(30))),
            // Interior point to make a curve
            List.of(new Translation2d(0.75, -0.5)),
            // End 1 meter straight ahead and 0.75 meters right of where we started, facing the +X direction
            new Pose2d(1, -0.75, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }

  public Trajectory getBlueParkAfterRightNoteRightPathTrajectory(TrajectoryConfig config){
    Trajectory parkTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing 30 degrees right
            new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(-30))),
            // Interior point to make a curve
            List.of(new Translation2d(0.75, 0.5)),
            // End 1 meter straight ahead and 0.75 meters left of where we started, facing the +X direction
            new Pose2d(1, 0.75, new Rotation2d(Math.toRadians(0))),
            config);

    return parkTrajectory;
  }
}
