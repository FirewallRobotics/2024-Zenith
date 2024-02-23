// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AxleConstants;
import frc.robot.subsystems.AxleSubsystem;

public class AngleForAmpCommand extends Command {

  private final AxleSubsystem m_AxleSubsystem;
  private double neededRadians = AxleConstants.kTestRadiansNeeded;

  /** Creates a new AngleForAmpCommand. */
  public AngleForAmpCommand(AxleSubsystem a_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_AxleSubsystem = a_Subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_AxleSubsystem.getAngle() > neededRadians) {
      m_AxleSubsystem.AxleDown();
    } else if (m_AxleSubsystem.getAngle() < neededRadians) {
      m_AxleSubsystem.AxleUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_AxleSubsystem.getAngle() == neededRadians) {
      return true;
    }
    return false;
  }
}
