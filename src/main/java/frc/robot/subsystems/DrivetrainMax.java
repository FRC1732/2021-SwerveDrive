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
import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainMax extends SubsystemBase {
  private final SwerveModuleMax frontLeft = new SwerveModuleMax(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE,
  Constants.DRIVETRAIN_FRONT_LEFT_AZIMUTH, Constants.DRIVETRAIN_FRONT_LEFT_ALIGNMENT_CHANNEL,
  Constants.DRIVETRAIN_FRONT_LEFT_ALIGNMENT_TARGET, SwervePosition.FrontLeft);

  private final SwerveModuleMax frontRight = new SwerveModuleMax(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE,
  Constants.DRIVETRAIN_FRONT_RIGHT_AZIMUTH, Constants.DRIVETRAIN_FRONT_LEFT_ALIGNMENT_CHANNEL,
  Constants.DRIVETRAIN_FRONT_RIGHT_ALIGNMENT_TARGET, SwervePosition.FrontRight);

  private final SwerveModuleMax backLeft = new SwerveModuleMax(Constants.DRIVETRAIN_BACK_LEFT_DRIVE,
  Constants.DRIVETRAIN_BACK_LEFT_AZIMUTH, Constants.DRIVETRAIN_FRONT_LEFT_ALIGNMENT_CHANNEL,
  Constants.DRIVETRAIN_BACK_LEFT_ALIGNMENT_TARGET, SwervePosition.BackLeft);

  private final SwerveModuleMax backRight = new SwerveModuleMax(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE,
  Constants.DRIVETRAIN_BACK_RIGHT_AZIMUTH, Constants.DRIVETRAIN_BACK_RIGHT_ALIGNMENT_CHANNEL,
  Constants.DRIVETRAIN_BACK_RIGHT_ALIGNMENT_TARGET, SwervePosition.BackRight);

  private final Gyro gyro = new AHRS(SPI.Port.kMXP);

  public DrivetrainMax() {
    gyro.reset();
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
    speed = speed/2d;
    Rotation2d angle = new Rotation2d(forward, -1d * strafe);
    SwerveModuleState state = new SwerveModuleState(speed, angle);
    frontLeft.setDesiredState(state);
    frontRight.setDesiredState(state);
    backLeft.setDesiredState(state);
    backRight.setDesiredState(state);
  }

  // Stops the motors from
  public void stop() {
    drive(0, 0, 0, false);
  }

  public boolean setStartPosition() {
    // return true when all modules report aligned.
    boolean retval = true;
    retval &= frontLeft.setStartPosition();
    retval &= frontRight.setStartPosition();
    retval &= backLeft.setStartPosition();
    retval &= backRight.setStartPosition();

    if (retval) {
      gyro.reset();
    }

    return retval;
  }
}
