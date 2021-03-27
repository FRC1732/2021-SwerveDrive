// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule {
  private final SwervePosition m_position;
  private static final double kWheelRadius = 0.0613;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final CANEncoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  private ShuffleboardTab tab = Shuffleboard.getTab("Swerve Modules");

  private double driveOutput;
  private double driveFeedforward;
  private double turnOutput;
  private double turnFeedforward;
  private SwerveModuleState optimizedState;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int talonID, int sparkID, SwervePosition position) {
    m_position = position;
    m_driveMotor = new TalonFX(talonID);
    m_turningMotor = new CANSparkMax(sparkID, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius /
    // kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    m_turningEncoder = m_turningMotor.getEncoder(); // We think this is the quadature with 4096 res
    m_turningEncoder.setPositionConversionFactor(2 * Math.PI / kEncoderResolution);
    m_turningEncoder.setPosition(0);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    optimizedState = new SwerveModuleState();

    initShuffleBoard(position);
  }

  private void initShuffleBoard(SwervePosition position) {
    // String prefix = position.toString();
    ShuffleboardLayout layout = tab.getLayout(position.toString(), BuiltInLayouts.kList).withSize(2, 4);

    layout.addNumber("Encoder Position", m_turningEncoder::getPosition).withPosition(0, 0);
    layout.addNumber("Encoder Speed", m_turningMotor::get).withPosition(0, 1);
    layout.addNumber("Encoder Output", this::getTurnOutput).withPosition(0, 2);
    layout.addNumber("Encoder Feed Forward", this::getTurnFeedforward).withPosition(0, 3);
    layout.addString("State", () -> this.getState().toString()).withPosition(0, 4);
    layout.addNumber("Encoder Target", () -> optimizedState.angle.getRadians()).withPosition(0, 5);
    // layout.addNumber("Encoder Speed", m_turningMotor::get).withPosition(0, 1);

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity(),
        new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity(),
    optimizedState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(optimizedState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition(), optimizedState.angle.getRadians());

    turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    // m_driveMotor.set(ControlMode.PercentOutput, driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public CANSparkMax getTurningMotor() {
    return m_turningMotor;
  }

  public double getDriveOutput() {
    return driveOutput;
  }

  public double getDriveFeedforward() {
    return driveFeedforward;
  }

  public double getTurnOutput() {
    return turnOutput;
  }

  public double getTurnFeedforward() {
    return turnFeedforward;
  }

}
