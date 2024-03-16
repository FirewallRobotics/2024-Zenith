// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseShooterCommand extends Command {
  /** Creates 0-a new ReverseShooterCommand. */
  private ShooterSubsystem m_Shooter;

  private IntakeSubsystem m_Intake;
  private LEDSubsystem m_LED;

  public ReverseShooterCommand(
      ShooterSubsystem s_Subsystem, IntakeSubsystem i_Subsystem, LEDSubsystem l_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Shooter = s_Subsystem;

    addRequirements(s_Subsystem);
    addRequirements(i_Subsystem);
    addRequirements(l_Subsystem);
  }

  public void sensorReverseShooter() {
    if (m_Intake.outputSensor.get() == IntakeConstants.kIntakeSensorNoteDetected) {
      System.out.println("Note out of place!");
      m_Intake.StartReverseIndex();
      m_Shooter.ReverseShooter();
    } else {
      m_Intake.StopIntake();
      m_Shooter.StopShoot();
      System.out.println("Note ready!");
      // m_LED.SetOrange();
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensorReverseShooter();
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
