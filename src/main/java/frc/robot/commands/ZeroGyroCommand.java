package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TBSwerve_DriveSubsystem;

public final class ZeroGyroCommand extends InstantCommand {

  private static final TBSwerve_DriveSubsystem DRIVE = RobotContainer.DRIVE;

  public ZeroGyroCommand() {
    addRequirements(DRIVE);
  }

  @Override
  public void initialize() {
    DRIVE.zeroGyro();
  }
}
