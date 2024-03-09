// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AxleSubsystem;

public class AxleEncoderTestCommand extends Command {
  /** Creates a new AxleEncoderTestCommand. */
  AxleSubsystem m_axle;

  public AxleEncoderTestCommand(AxleSubsystem a_Subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_axle = a_Subsystem;

    addRequirements(a_Subsystem);
  }

  Timer timer = new Timer();
  double axleSpeed;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    axleSpeed = 0.06;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() > 2) {
      timer.reset();

      axleSpeed += 0.001;

      System.out.println("Current Axle Speed: " + axleSpeed);
    }

    System.out.println("Time: " + timer.get());

    m_axle.setAxleMotorSpeed(axleSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_axle.setAxleMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
