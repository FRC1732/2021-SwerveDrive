package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.control.DriverControls;
import frc.robot.subsystems.TBSwerve_DriveSubsystem;

public final class TeleOpDriveCommand extends CommandBase {

  private static final double DEADBAND = 0.05;
  private static final TBSwerve_DriveSubsystem DRIVE = RobotContainer.DRIVE;
  private static final DriverControls controls = RobotContainer.CONTROLS.getDriverControls();

  public TeleOpDriveCommand() {
    addRequirements(DRIVE);
  }

  @Override
  public void execute() {
    double forward = deadband(controls.getForward());
    double strafe = deadband(controls.getStrafe());
    double yaw = deadband(controls.getYaw());

    DRIVE.drive(-forward, -strafe, -yaw);
  }

  @Override
  public void end(boolean interrupted) {
    DRIVE.drive(0.0, 0.0, 0.0);
  }

  private double deadband(double value) {
    if (Math.abs(value) < DEADBAND) return 0.0;
    return value;
  }
}
