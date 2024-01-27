// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AxleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAmpCommand extends Command {
  /** Creates a new ShootSpeakerCommand. */
  private final ShooterSubsystem m_Shooter;

  private final AxleSubsystem m_Axle;

  public ShootAmpCommand(ShooterSubsystem sh_Subsystem, AxleSubsystem ss_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Shooter = sh_Subsystem;
    m_Axle = ss_Subsystem;

    addRequirements(sh_Subsystem);
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
