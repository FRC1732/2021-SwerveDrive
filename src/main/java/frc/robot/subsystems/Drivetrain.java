// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.SwerveModule;
import frc.robot.drivers.SwerveModuleMax;
import frc.robot.drivers.SwervePosition;
import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  private static final double TRACKWIDTH = 0.3854; // in meters
  private static final double WHEELBASE = 0.3854; // in meters

  // Translation2Ds are in meters
  private final Translation2d frontLeftLocation = new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / -2.0);
  private final Translation2d frontRightLocation = new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0);
  private final Translation2d backLeftLocation = new Translation2d(TRACKWIDTH / -2.0, WHEELBASE / -2.0);
  private final Translation2d backRightLocation = new Translation2d(TRACKWIDTH / -2.0, WHEELBASE / 2.0);

  private final Gyro gyro = new AHRS(SPI.Port.kMXP);

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

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
      backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d());

  public Drivetrain() {
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
    // joy stick values to velocity values; multiple by MAX'es
    // Maybe this method should take in velocity values instead of joystick values?
    double adjustedForward = forward * Constants.MAX_SPEED;
    double adjustedStrafe = strafe * Constants.MAX_SPEED;
    double adjustedRotate = rotate * Constants.MAX_ANGULAR_VELOCITY;

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(adjustedForward, adjustedStrafe, adjustedRotate));

    // This will make sure speeds do not exceed maxium and adjust all wheels if
    // necessary.
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(gyro.getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
        backRight.getState());
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
