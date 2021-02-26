package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public final class ZeroGyroCommand extends InstantCommand {

  private static final DriveSubsystem DRIVE = RobotContainer.DRIVE;

  public ZeroGyroCommand() {
    Subsystem DriveMode;
    addRequirements(DriveMode);
  }

  @Override
  public void initialize() {
    DRIVE.zeroGyro();
  }
}
