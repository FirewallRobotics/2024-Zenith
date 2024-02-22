// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AxleSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbLeftCommand extends Command {
  /** Creates a new ShootSpeakerCommand. */
  private final ClimbSubsystem m_Climb;

  private final AxleSubsystem m_Axle;

  /** Creates a new ClimbDefaultCommand. */
  public ClimbLeftCommand(ClimbSubsystem c_Subsystem, AxleSubsystem a_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Climb = c_Subsystem;
    m_Axle = a_Subsystem;

    addRequirements(c_Subsystem);
    addRequirements(a_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Climb.ClimbLeft();
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
    if (m_Climb.bottomLimitSwitch.isPressed()) {
      return true;
    }
    return false;
  }
}
