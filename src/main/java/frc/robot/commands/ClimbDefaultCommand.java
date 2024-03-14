// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AxleConstants;
import frc.robot.subsystems.AxleSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbDefaultCommand extends Command {
  private final ClimbSubsystem m_Climb;
  private final AxleSubsystem m_Axle;

  /** Creates a new ClimbDefaultCommand. */
  public ClimbDefaultCommand(ClimbSubsystem c_Subsystem, AxleSubsystem a_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Climb = c_Subsystem;
    m_Axle = a_Subsystem;

    addRequirements(c_Subsystem);
    addRequirements(a_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Climb.keepDown = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_Axle.AxleEncoder.getPosition() - AxleConstants.kDefaultHeight) < 10) {
      m_Climb.DefaultHeight();
    } else {
      m_Climb.stopClimb();
      ;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted == true) {
      m_Climb.stopClimb();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Test if currentPos is equal to the default height
    if (m_Climb.topLimitSwitch.isPressed()
        || m_Axle.reachedSetPosition(AxleConstants.kAmpHeight, 0.01)) {
      return true;
    }
    return false;
  }
}
