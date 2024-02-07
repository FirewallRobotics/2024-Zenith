// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoAimConstants;

public class AutoAimSubsystem extends SubsystemBase {
  /** Creates a new AutoAimSubsystem. */
  private double launchStartingHeight = AutoAimConstants.kLaunchStartingHeight;

  private double launchToCameraDifference = AutoAimConstants.kLaunchToCameraDifference;

  private double targetX = AutoAimConstants.kTargetX;
  private double targetY = AutoAimConstants.kTargetY;

  private double launchVelocity = AutoAimConstants.kLaunchVelocity;
  private double gravitationalConstant = AutoAimConstants.kgravitationalConstant;

  private double velocityConstant; // Calculated in constructor

  private int maxIterations = AutoAimConstants.kMaxIterations;
  private final double rangeForAimAngle = AutoAimConstants.kRangeForAimAngle;
  private final double rangeForMax = AutoAimConstants.kRangeForMax;

  private double cameraDistance;
  private double launchDistance;

  public AutoAimSubsystem() {
    launchStartingHeight = 0.35;
    targetX = 0;
    targetY = 2;
    launchVelocity = 9;

    cameraDistance =
        1.0 - launchToCameraDifference; // Default value is 1.0 - positive difference between
    // cameraX and launchX
    launchDistance = 1.0; // Default value is 1.0

    velocityConstant = gravitationalConstant / (2 * Math.pow(launchVelocity, 2));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLaunchDistance(double cameraDistance) {
    this.cameraDistance = cameraDistance;
    this.launchDistance = cameraDistance + launchToCameraDifference;
  }

  private double plugInTheta(double theta) {
    double distance = targetX + launchDistance;

    return launchStartingHeight
        + (distance * Math.tan(theta))
        - (velocityConstant * Math.pow(distance, 2) / Math.pow(Math.cos(theta), 2));
  }

  private double getMinAngle() {
    return Math.atan2(targetY - launchStartingHeight, targetX + launchDistance);
  }

  private double plugInThetaDerivative(double theta) {
    double distance = targetX + launchDistance;

    return (distance / Math.pow(Math.cos(theta), 2))
        - (velocityConstant
            * Math.pow(distance, 2)
            * 2
            * Math.tan(theta)
            / Math.pow(Math.cos(theta), 2));
  }

  public double SolveForAngle(double launchDistance) {
    double minAngle = getMinAngle();
    System.out.println("MinAngle" + minAngle);

    double thetaOfMaximum = solveForThetaOfMaximum(minAngle);

    if (plugInTheta(thetaOfMaximum) < targetY) {
      System.out.println("NotInRange");
      return -1;
    }

    double maxAngle = thetaOfMaximum;
    double midAngle = (minAngle + maxAngle) / 2;

    for (int i = 0; i < maxIterations; i++) {
      midAngle = (minAngle + maxAngle) / 2;

      double midValue = plugInTheta(midAngle);

      if (Math.abs(midValue - targetY) < rangeForAimAngle) {
        System.out.println("Found aim angle: " + midAngle);
        return midAngle;
      }

      if (midValue - targetY < 0) {
        minAngle = midAngle;
      } else {
        maxAngle = midAngle;
      }
    }

    System.out.println("Failed Finding Aim Angle | Final Value: " + midAngle);
    return midAngle;
  }

  private double solveForThetaOfMaximum(double minAngle) {
    double maxAngle = Math.PI / 2;
    double midAngle = minAngle;

    for (int i = 0; i < maxIterations; i++) {
      midAngle = (minAngle + maxAngle) / 2;

      double midValue = plugInThetaDerivative(midAngle);

      if (Math.abs(midValue) < rangeForMax) {
        System.out.println("Found Angle for Max: " + midAngle);
        return midAngle;
      }

      if (midValue > 0) {
        minAngle = midAngle;
      } else {
        maxAngle = midAngle;
      }
    }

    System.out.println("Failed Angle for Max | Final Value: " + midAngle);
    return midAngle;
  }
}
