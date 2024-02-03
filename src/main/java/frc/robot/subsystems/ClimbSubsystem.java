// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climbConstants;

public class ClimbSubsystem extends SubsystemBase {

  public static CANSparkMax climbMotorRight;
  public static CANSparkMax climbMotorLeft;

  private SparkLimitSwitch m_rightLimit;
  private SparkLimitSwitch m_leftLimit;

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

    m_rightLimit = climbMotorRight.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    m_leftLimit = climbMotorLeft.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    m_rightLimit.enableLimitSwitch(true);
    m_leftLimit.enableLimitSwitch(true);

    SmartDashboard.putBoolean("Right Limit Enabled", m_rightLimit.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Left Limit Enabled", m_leftLimit.isLimitSwitchEnabled());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // enable/disable limit switches based on value read from SmartDashboard
    m_rightLimit.enableLimitSwitch(SmartDashboard.getBoolean("Right Limit Enabled", false));
    m_leftLimit.enableLimitSwitch(SmartDashboard.getBoolean("Left Limit Enabled", false));

    /**
     * The isPressed() method can be used on a SparkLimitSwitch object to read the state of the
     * switch.
     *
     * <p>In this example, the polarity of the switches are set to normally closed. In this case,
     * isPressed() will return true if the switch is pressed. It will also return true if you do not
     * have a switch connected. isPressed() will return false when the switch is released.
     */
    SmartDashboard.putBoolean("Right Limit Switch", m_rightLimit.isPressed());
    SmartDashboard.putBoolean("Left Limit Switch", m_leftLimit.isPressed());
  }

  public void DefaultHeight() {}

  public void ClimbLeft() {}

  public void ClimbMiddle() {}

  public void ClimbRight() {}
}
