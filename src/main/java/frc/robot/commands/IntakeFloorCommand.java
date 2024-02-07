// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AxleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFloorCommand extends Command {
  /** Creates a new ShootSpeakerCommand. */
  private final IntakeSubsystem m_Intake;

  private final AxleSubsystem m_Seesaw;

  public IntakeFloorCommand(IntakeSubsystem i_Subsystem, AxleSubsystem ss_Subsystem) {
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
  public void execute() {
    m_Seesaw.IntakeFloorAngle();

    // This is the LED sensor.
    m_Intake.sensorStartIntake();

    m_Intake.StartIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted == true) {
      m_Intake.StopIntake();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_Intake.StopIntake();
    return false;
  }
}
