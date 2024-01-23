// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SeesawSubsystem;

public class IntakeFloorCommand extends Command {
  /** Creates a new ShootSpeakerCommand. */

  private final IntakeSubsystem m_Intake;
  private final SeesawSubsystem m_Seesaw;

  public IntakeFloorCommand(IntakeSubsystem i_Subsystem, SeesawSubsystem ss_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Intake = i_Subsystem;
    m_Seesaw = ss_Subsystem;

    addRequirements(i_Subsystem);
    addRequirements(ss_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
