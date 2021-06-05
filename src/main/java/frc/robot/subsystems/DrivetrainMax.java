// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.SwerveModuleMax;
import frc.robot.Constants;
import frc.robot.drivers.SwervePosition;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainMax extends SubsystemBase {
  private final SwerveModuleMax frontLeft = new SwerveModuleMax(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE,
  Constants.DRIVETRAIN_FRONT_LEFT_AZIMUTH, Constants.DRIVETRAIN_FRONT_LEFT_ALIGNMENT_CHANNEL,
  Constants.DRIVETRAIN_FRONT_LEFT_ALIGNMENT_TARGET, SwervePosition.BackLeft);// new SwerveModuleMax(9,4,0,0,null);

  public DrivetrainMax() {
  }

  /**
   * Joystick inputs to drive the robot.
   * 
   * @param forward       Forward (1) or backward (-1) speed with zero being no
   *                      movement
   * @param strafe        Left (1) or right (-1) movement control with zero being
   *                      no strafe
   * @param rotate        CCW (1) or CW (-1) rotation with zero being no rotation
   * @param fieldRelative Used with Odometry, set true in autonmous mode, false
   *                      otherwise
   */
  public void drive(double forward, double strafe, double rotate, boolean fieldRelative) {
    double speed = Math.hypot(strafe, forward);
    speed = speed > 1 ? 1 : speed;
    Rotation2d angle = new Rotation2d(forward, strafe);
    SwerveModuleState state = new SwerveModuleState(speed, angle);
    frontLeft.setDesiredState(state);
  }

  // Stops the motors from
  public void stop() {
    drive(0, 0, 0, false);
  }
}
