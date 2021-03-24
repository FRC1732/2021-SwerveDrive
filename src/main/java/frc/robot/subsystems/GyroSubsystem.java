// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GyroSubsystem extends SubsystemBase {
  private AHRS ahrs;
  private ShuffleboardTab tab = Shuffleboard.getTab("Gyro Control Test");
  private final boolean IS_VERBOSE = false;

  /** Creates a new GyroSubsystem. */
  public GyroSubsystem() {
    this.ahrs = new AHRS(SPI.Port.kMXP);

    tab.getComponents().clear();

    /* Display 6-axis Processed Angle Data */
    tab.addBoolean("IMU_Connected", ahrs::isConnected).withPosition(1, 1);
    tab.addBoolean("IMU_IsCalibrating", ahrs::isCalibrating).withPosition(2, 1);
    tab.addNumber("IMU_Yaw", ahrs::getYaw).withPosition(2, 1);
    tab.addNumber("IMU_Pitch", ahrs::getPitch).withPosition(2, 2);
    tab.addNumber("IMU_Roll", ahrs::getRoll).withPosition(2, 3);

    tab.addNumber("IMU_TotalYaw", ahrs::getAngle).withPosition(3,1);
    tab.addNumber("IMU_YawRateDPS", ahrs::getRate).withPosition(3,2);

    if (IS_VERBOSE) {
      /* Display tilt-corrected, Magnetometer-based heading (requires */
      /* magnetometer calibration to be useful) */

      tab.addNumber("IMU_CompassHeading", ahrs::getCompassHeading);

      /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
      tab.addNumber("IMU_FusedHeading", ahrs::getFusedHeading);

      /* These functions are compatible w/the WPI Gyro Class, providing a simple */
      /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

      //tab.addNumber("IMU_TotalYaw", ahrs::getAngle);
      //tab.addNumber("IMU_YawRateDPS", ahrs::getRate);

      /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

      tab.addNumber("IMU_Accel_X", ahrs::getWorldLinearAccelX);
      tab.addNumber("IMU_Accel_Y", ahrs::getWorldLinearAccelY);
      tab.addBoolean("IMU_IsMoving", ahrs::isMoving);
      tab.addBoolean("IMU_IsRotating", ahrs::isRotating);

      /* Display estimates of velocity/displacement. Note that these values are */
      /* not expected to be accurate enough for estimating robot position on a */
      /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
      /* of these errors due to single (velocity) integration and especially */
      /* double (displacement) integration. */

      tab.addNumber("Velocity_X", ahrs::getVelocityX);
      tab.addNumber("Velocity_Y", ahrs::getVelocityY);
      tab.addNumber("Displacement_X", ahrs::getDisplacementX);
      tab.addNumber("Displacement_Y", ahrs::getDisplacementY);

      /* Display Raw Gyro/Accelerometer/Magnetometer Values */
      /* NOTE: These values are not normally necessary, but are made available */
      /* for advanced users. Before using this data, please consider whether */
      /* the processed data (see above) will suit your needs. */

      tab.addNumber("RawGyro_X", ahrs::getRawGyroX);
      tab.addNumber("RawGyro_Y", ahrs::getRawGyroY);
      tab.addNumber("RawGyro_Z", ahrs::getRawGyroZ);
      tab.addNumber("RawAccel_X", ahrs::getRawAccelX);
      tab.addNumber("RawAccel_Y", ahrs::getRawAccelY);
      tab.addNumber("RawAccel_Z", ahrs::getRawAccelZ);
      tab.addNumber("RawMag_X", ahrs::getRawMagX);
      tab.addNumber("RawMag_Y", ahrs::getRawMagY);
      tab.addNumber("RawMag_Z", ahrs::getRawMagZ);
      tab.addNumber("IMU_Temp_C", ahrs::getTempC);

      /* Sensor Board Information */
      tab.addString("FirmwareVersion", ahrs::getFirmwareVersion);

      /* Quaternion Data */
      /* Quaternions are fascinating, and are the most compact representation of */
      /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
      /* from the Quaternions. If interested in motion processing, knowledge of */
      /* Quaternions is highly recommended. */
      tab.addNumber("QuaternionW", ahrs::getQuaternionW);
      tab.addNumber("QuaternionX", ahrs::getQuaternionX);
      tab.addNumber("QuaternionY", ahrs::getQuaternionY);
      tab.addNumber("QuaternionZ", ahrs::getQuaternionZ);

      /* Connectivity Debugging Support */
      tab.addNumber("IMU_Byte_Count", ahrs::getByteCount);
      tab.addNumber("IMU_Update_Count", ahrs::getUpdateCount);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void reset() {
    ahrs.reset();
  }

  public double yaw() {
    return ahrs.getYaw();
  }

  public double pitch() {
    return ahrs.getPitch();
  }

  public double roll() {
    return ahrs.getRoll();
  }

  public double angle(){
    return ahrs.getAngle();
  }

  public double rate() {
    return ahrs.getRate();
  }
  
}
