// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  SwerveModule frontLeft = new SwerveModule(1, 5);
  SwerveModule frontRight = new SwerveModule(2, 6);
  SwerveModule backLeft = new SwerveModule(3, 7);
  SwerveModule backRight = new SwerveModule(4, 8);

  final double LENGTH = 0;
  final double WIDTH = 0;
  double r = Math.sqrt((LENGTH*LENGTH)+(WIDTH*WIDTH));

  
  /** Creates a new Drivetrain. */
  public Drivetrain() {}

  private void drive(double x1, double y, double x2){
    y *= -1;

    double a = x1 - x2 * (LENGTH / r);
    double b = x1 + x2 * (LENGTH / r);
    double c = y - x2 * (WIDTH / r);
    double d = y + x2 * (WIDTH / r);

    double backRightSpeed = Math.sqrt(a*a + d*d);
    double backLeftSpeed  = Math.sqrt(a*a + c*c);
    double frontRightSpeed = Math.sqrt(b*b + d*d);
    double frontLeftSpeed = Math.sqrt(b*b + c*c);

    double backRightAngle = Math.atan2(a,d) / Math.PI;
    double backLeftAngle = Math.atan2(a, c) / Math.PI;
    double frontRightAngle = Math.atan2(b, d) / Math.PI;
    double frontLeftAngle = Math.atan2(b, c) / Math.PI;

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}


