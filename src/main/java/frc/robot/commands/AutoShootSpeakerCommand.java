// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootSpeakerCommand extends Command {
  /** Creates a new ShootSpeakerCommand. */
  
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;

  final Timer timer = new Timer();

  public AutoShootSpeakerCommand(ShooterSubsystem s_Subsystem, IntakeSubsystem i_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_shooter = s_Subsystem;
    m_intake = i_Subsystem;

    addRequirements(s_Subsystem);
    addRequirements(i_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.ShootSpeaker();

    if(timer.hasElapsed(0.5)){
      m_intake.StartIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.StopShoot();
    m_intake.StopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.75);
  }
}
