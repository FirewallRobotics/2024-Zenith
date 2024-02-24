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

  public static CANSparkMax climbMotorMaster;
  // public static CANSparkMax climbMotorLeft;

  public SparkLimitSwitch topLimitSwitch;
  public SparkLimitSwitch bottomLimitSwitch;

  // Variables for setting the climb to default

  // public double hieghtDefault;
  public double currentPos;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {

    climbMotorMaster =
        new CANSparkMax(
            climbConstants.kClimbMotorPort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);

    topLimitSwitch = climbMotorMaster.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    bottomLimitSwitch =
        climbMotorMaster.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

    topLimitSwitch.enableLimitSwitch(true);
    bottomLimitSwitch.enableLimitSwitch(true);

    SmartDashboard.putBoolean("Top Limit Enabled", topLimitSwitch.isLimitSwitchEnabled());
    SmartDashboard.putBoolean("Bottom Limit Enabled", bottomLimitSwitch.isLimitSwitchEnabled());

    climbMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    climbMotorMaster.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    climbMotorMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);
    climbMotorMaster.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
  }

  /*
  public void initialize(){

    hieghtDefault = climbEncoder.getPosition();
  }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // enable/disable limit switches based on value read from SmartDashboard
    topLimitSwitch.enableLimitSwitch(SmartDashboard.getBoolean("Top Limit Enabled", true));
    bottomLimitSwitch.enableLimitSwitch(SmartDashboard.getBoolean("Bottom Limit Enabled", true));

    /**
     * The isPressed() method can be used on a SparkLimitSwitch object to read the state of the
     * switch.
     *
     * <p>In this example, the polarity of the switches are set to normally closed. In this case,
     * isPressed() will return true if the switch is pressed. It will also return true if you do not
     * have a switch connected. isPressed() will return false when the switch is released.
     */
    SmartDashboard.putBoolean("Top Limit Switch", topLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Bottom Limit Switch", topLimitSwitch.isPressed());

    //
    if (topLimitSwitch.isPressed() || bottomLimitSwitch.isPressed()) {
      stopClimb();
    }
  }

  public void DefaultHeight() {
    if (topLimitSwitch.isPressed()) {
      stopClimb();
    } else {
      climbMotorMaster.set(-1 * climbConstants.kClimbMotorPortSpeed);
    }
  }

  public void ClimbLeft() {
    if (bottomLimitSwitch.isPressed()) {
      stopClimb();
    } else {
      climbMotorMaster.set(climbConstants.kClimbMotorPortSpeed);
    }
  }

  public void ClimbMiddle() {
    if (bottomLimitSwitch.isPressed()) {
      stopClimb();
    } else {
      climbMotorMaster.set(climbConstants.kClimbMotorPortSpeed);
    }
  }

  public void ClimbRight() {
    if (bottomLimitSwitch.isPressed()) {
      stopClimb();
    } else {
      climbMotorMaster.set(climbConstants.kClimbMotorPortSpeed);
    }
  }

  // Stop climb
  public void stopClimb() {
    climbMotorMaster.set(0);
  }
}
