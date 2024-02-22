// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AxleSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class AxleDownCommand extends Command {
  /** Creates a new ShootSpeakerCommand. */
  private final AxleSubsystem m_Axle;

  private final ClimbSubsystem m_Climb;

  public AxleDownCommand(AxleSubsystem a_Subsystem, ClimbSubsystem c_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Axle = a_Subsystem;
    m_Climb = c_Subsystem;

    addRequirements(a_Subsystem);
    addRequirements(c_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Climb.bottomLimitSwitch.isPressed()) {
      m_Axle.AxleUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
