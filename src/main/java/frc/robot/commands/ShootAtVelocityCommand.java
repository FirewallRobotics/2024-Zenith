// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAtVelocityCommand extends Command {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;

  /** Creates a new ShootAtVelocityCommand. */
  public ShootAtVelocityCommand(ShooterSubsystem sh_Subsystem, IntakeSubsystem i_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ShooterSubsystem = sh_Subsystem;
    m_IntakeSubsystem = i_Subsystem;

    addRequirements(sh_Subsystem);
    addRequirements(i_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ShooterSubsystem.ShootSpeakerWithPID();

    // We may not get to the PID value before we need to shoot.
    if (ShooterSubsystem.ShooterEncoder.getVelocity() >= (m_ShooterSubsystem.finalVelocity - 2.0)) {
      m_IntakeSubsystem.StartIndex();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.StopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
