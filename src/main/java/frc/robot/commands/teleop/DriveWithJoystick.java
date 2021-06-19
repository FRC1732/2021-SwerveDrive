// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainMax;

public class DriveWithJoystick extends CommandBase {
  private Joystick leftJoystick;
  private Joystick rightJoystick;
  private Boolean fieldCentric;
  private Drivetrain drivetrain;

  // Slew rate limiters to make joystick inputs more gentle; 1/4 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(4);

  public DriveWithJoystick(Joystick leftJoystick, Joystick rightJoystick, Drivetrain drivetrain, Boolean fieldCentric) {
    addRequirements(drivetrain);
    
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.drivetrain = drivetrain;
    this.fieldCentric = fieldCentric;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // flip +/- to match drivetrain.drive
    double forward = Math.signum(leftJoystick.getRawAxis(1)) * -Math.pow(leftJoystick.getRawAxis(1),2);
    double strafe = Math.signum(leftJoystick.getRawAxis(0)) * Math.pow(leftJoystick.getRawAxis(0),2);
    double rotation = Math.signum(rightJoystick.getRawAxis(0)) * Math.pow(rightJoystick.getRawAxis(0),2);

    forward = Math.abs(forward) < 0.05 ? 0.0 : forward;
    strafe = Math.abs(strafe) < 0.05 ? 0.0 : strafe;
    rotation = Math.abs(rotation) < 0.05 ? 0.0 : rotation;

    drivetrain.drive(xspeedLimiter.calculate(forward), yspeedLimiter.calculate(strafe), rotation, false); //xspeedLimiter.calculate(forward), yspeedLimiter.calculate(strafe)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
