// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climbConstants;

public class ClimbSubsystem extends SubsystemBase {

  public static CANSparkMax climbMotorRight;

  public static CANSparkMax climbMotorLeft;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {

    climbMotorRight =
        new CANSparkMax(
            climbConstants.kRightClimbMotorPort,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    climbMotorLeft =
        new CANSparkMax(
            climbConstants.kLeftClimbMotorPort,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void DefaultHeight() {}

  public void ClimbLeft() {}

  public void ClimbMiddle() {}

  public void ClimbRight() {}
}
