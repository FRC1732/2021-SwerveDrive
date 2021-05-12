// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class DriveWithJoystick extends CommandBase {
  private int counter = 0;
  private Timer timer;

  private Joystick leftJoystick;
  //private Joystick rightJoystick;
  private Boolean fieldCentric;
  private Drivetrain m_swerve;
  private Intake intake;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  public DriveWithJoystick(Joystick leftJoystick, Drivetrain drivetrain, Boolean fieldCentric, Intake intake) {
    addRequirements(drivetrain);
   // addRequirements(intake);
    
    this.leftJoystick = leftJoystick;
    //this.rightJoystick = rightJoystick;
    this.m_swerve = drivetrain;
    this.fieldCentric = fieldCentric;
    this.intake = intake;

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

    double forward = leftJoystick.getRawAxis(1);
    double strafe = leftJoystick.getRawAxis(0);
    double rotation = leftJoystick.getRawAxis(4);
    
    boolean intakeOn = leftJoystick.getTrigger();

    forward = Math.abs(forward) < 0.05? 0.0: forward;
    strafe = Math.abs(strafe) < 0.05? 0.0: strafe;
    rotation = Math.abs(rotation) < 0/05? 0.0: rotation;

    m_swerve.drive(forward, strafe, rotation, true);
    intake.takeIn(intakeOn);

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
