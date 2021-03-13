/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
  private Drivetrain drivetrain;
  private double distance;
  private double leftStart;
  private double rightStart;
  private double scale = 1.0;
  private final double TICKS_PER_FOOT = 3.33333333;
  /**
   * Creates a new DriveDistance.
   * @param distance the distance in feet
   */
  public DriveDistance(Drivetrain drivetrain, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.distance = distance /= TICKS_PER_FOOT;
    this.drivetrain = drivetrain;
  }
// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // leftStart = drivetrain.getLeftEncoder();
    // rightStart = drivetrain.getRightEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0.25, 0.25, 0, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return Math.abs(drivetrain.getLeftEncoder() - leftStart) > Math.abs(distance);
  }
}
