// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.climbConstants;

public class ClimbSubsystem extends SubsystemBase {

  public static CANSparkMax climbMotorMaster;
  // public static CANSparkMax climbMotorLeft;

  public static SparkLimitSwitch m_climbLimit;

  public static AbsoluteEncoder climbEncoder;

  // Variables for setting the climb to default

  // public double hieghtDefault;
  public double currentPos;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {

    climbMotorMaster =
        new CANSparkMax(
            climbConstants.kClimbMotorPort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    m_climbLimit = climbMotorMaster.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    m_climbLimit.enableLimitSwitch(true);

    SmartDashboard.putBoolean("Right Limit Enabled", m_climbLimit.isLimitSwitchEnabled());

    climbMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    climbMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    climbMotorMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);
    climbMotorMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    climbEncoder = climbMotorMaster.getAbsoluteEncoder(Type.kDutyCycle);
  }

  /*
  public void initialize(){

    hieghtDefault = climbEncoder.getPosition();
  }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    currentPos = climbEncoder.getPosition();
    // enable/disable limit switches based on value read from SmartDashboard
    m_climbLimit.enableLimitSwitch(SmartDashboard.getBoolean("Right Limit Enabled", false));

    /**
     * The isPressed() method can be used on a SparkLimitSwitch object to read the state of the
     * switch.
     *
     * <p>In this example, the polarity of the switches are set to normally closed. In this case,
     * isPressed() will return true if the switch is pressed. It will also return true if you do not
     * have a switch connected. isPressed() will return false when the switch is released.
     */
    SmartDashboard.putBoolean("Right Limit Switch", m_climbLimit.isPressed());

    if (m_climbLimit.isPressed()) {
      stopClimb();
    }
  }

  public void DefaultHeight() {
    climbMotorMaster.set(-1 * climbConstants.kClimbMotorPortSpeed);
  }

  public void ClimbLeft() {
    climbMotorMaster.set(climbConstants.kClimbMotorPortSpeed);
  }

  public void ClimbMiddle() {
    climbMotorMaster.set(climbConstants.kClimbMotorPortSpeed);
  }

  public void ClimbRight() {
    climbMotorMaster.set(climbConstants.kClimbMotorPortSpeed);
  }

  // Stop climb
  public void stopClimb() {
    climbMotorMaster.set(0);
  }
}
