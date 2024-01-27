// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AxleSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimAmpCommand extends Command {
  /** Creates a new ShootSpeakerCommand. */

  private final DriveSubsystem m_Drivetrain;
  private final VisionSubsystem m_Vision;
  private final AxleSubsystem m_Seesaw;

  public AimAmpCommand(DriveSubsystem dt_Subsystem, VisionSubsystem v_Subsystem, AxleSubsystem ss_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_Drivetrain = dt_Subsystem;
    m_Vision = v_Subsystem;
    m_Seesaw = ss_Subsystem;

    addRequirements(dt_Subsystem);
    addRequirements(v_Subsystem);
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
