// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class DriveTime extends CommandBase {
Drivetrain drivetrain;
boolean atTime = false;

/** Creates a new DriveTime. */
public DriveTime(Drivetrain drivetrain, double forward, double strafe, double rotate, double seconds) {
  new InstantCommand(() -> drivetrain.drive(forward, strafe, rotate, false), drivetrain)
    .andThen(new WaitCommand(seconds)
    .andThen(new InstantCommand(() -> drivetrain.stop())));
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted)
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atTime;
  }
}
