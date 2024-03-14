// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class CenterNoteCommand extends Command {
  /** Creates a new CenterNoteCommand. */

  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;

  public CenterNoteCommand(ShooterSubsystem s_Subsystem, IntakeSubsystem i_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = s_Subsystem;
    m_intake = i_Subsystem;

    addRequirements(s_Subsystem);
    addRequirements(i_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.ReverseShooter();
    m_intake.StartIndexSlow();
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
