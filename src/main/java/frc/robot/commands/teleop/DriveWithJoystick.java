// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoystick extends CommandBase {
  private int counter = 0;
  private Timer timer;

  private Joystick leftJoystick;
  private Joystick rightJoystick;
  private Boolean fieldCentric;
  private Drivetrain m_swerve;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public DriveWithJoystick(Joystick leftJoystick, Joystick rightJoystick, Drivetrain drivetrain, Boolean fieldCentric) {
    addRequirements(drivetrain);
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.m_swerve = drivetrain;
    this.fieldCentric = fieldCentric;

    timer = new Timer();
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double t1, t2, t3, t4, t5;

    t1 = timer.get();
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final double xSpeed = m_xspeedLimiter.calculate(rightJoystick.getY()) * Drivetrain.kMaxSpeed;

    t2 = timer.get();
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final double ySpeed = m_yspeedLimiter.calculate(rightJoystick.getX()) * Drivetrain.kMaxSpeed;

    t3 = timer.get();
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final double rot = m_rotLimiter.calculate(leftJoystick.getX()) * Drivetrain.kMaxAngularSpeed;

    t4 = timer.get();
    m_swerve.drive(xSpeed, ySpeed, rot, false);

    t5 = timer.get();
    if (counter++ > 100) {
      System.out.println(String.format("Timers (%f) (%f) (%f) (%f)", t2 - t1, t3 - t2, t4 - t3, t5 - t4));
      System.out.println(String.format("X/Y/Rot (%f) (%f) (%f)", xSpeed, ySpeed, rot));
      counter = 0;
    }

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
