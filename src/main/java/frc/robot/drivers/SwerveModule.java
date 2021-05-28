// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class SwerveModule extends AbstractSwerveModule {
  // Encoder returns revolutions; convert to radians; apply gear ratio
  private static final double TURN_ENCODER_CONVERSION = 1.0 * 2 * Math.PI / 53.3;

  private final TalonFX driveMotor;
  private final CANSparkMax turningMotor;
  private final CANEncoder turningEncoder;

  // no PID on drive, just direct control but will use a slew.
  // private final PIDController drivePIDController = new PIDController(1, 0, 0);
  private final ProfiledPIDController turningPIDController = new ProfiledPIDController(0.5, 0.1, 0.02,
      new TrapezoidProfile.Constraints(Constants.MAX_ANGULAR_VELOCITY, Constants.MAX_ANGULAR_ACCELERATION));

  // private final SimpleMotorFeedforward driveMotorFeedforward = new
  // SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward turnMotorFeedforward = new SimpleMotorFeedforward(1, 0.5);

  private SwerveModuleState optimizedState;
  private double driveMotorInput;
  private double driveMotorOutput;
  private double driveMotorFeedforward;
  private double turnOutput;
  private double turnFFVoltage;

  private DutyCycle dutyCycle;
  private final double wheelAlignment;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int talonID, int sparkID, int absoluteChannel, double wheelAlignment, SwervePosition position) {
    super(position);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.peakOutputForward = 1.0;
    talonConfig.peakOutputReverse = 1.0;
    talonConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    driveMotor = new TalonFX(talonID);
    driveMotor.configAllSettings(talonConfig, 100);
    driveMotor.setNeutralMode(NeutralMode.Coast);

    turningMotor = new CANSparkMax(sparkID, MotorType.kBrushless);
    turningMotor.restoreFactoryDefaults();
    turningEncoder = turningMotor.getEncoder();
    turningEncoder.setPositionConversionFactor(TURN_ENCODER_CONVERSION);
    turningEncoder.setPosition(0);

    optimizedState = new SwerveModuleState();
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    dutyCycle = new DutyCycle(new DigitalInput(absoluteChannel));
    this.wheelAlignment = wheelAlignment;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // optimize will cause the modules to flip when going past +/-90 degrees
    // (East/West). I am not sure we want that but its going to be a driver decision
    // optimizedState = SwerveModuleState.optimize(desiredState, new
    // Rotation2d(turningEncoder.getPosition()));
    optimizedState = desiredState; // no optimization

    driveMotorInput = convertMetersPerSecondToMotorVelocity(optimizedState.speedMetersPerSecond);
    // driveMotor.set(ControlMode.Velocity, driveMotorInput);
    driveMotor.set(ControlMode.PercentOutput, driveMotorInput / MAX_VELOCITY_PER_100MS);

    turnOutput = turningPIDController.calculate(turningEncoder.getPosition(), optimizedState.angle.getRadians());
    turnFFVoltage = turnMotorFeedforward.calculate(turningPIDController.getSetpoint().velocity);

    turningMotor.setVoltage(turnOutput + turnFFVoltage);
  }

  @Override
  double getTurnPosition() {
    return turningEncoder.getPosition();
  }

  @Override
  double getTurnTarget() {
    return optimizedState.angle.getRadians();
  }

  @Override
  double getTurnMotorVoltage() {
    return turningMotor.getBusVoltage() * turningMotor.get();
  }

  @Override
  double getTurnMotorOutputLevel() {
    return turnOutput;
  }

  @Override
  double getTurnMotorFeedForward() {
    return turnFFVoltage;
  }

  @Override
  double getDrivePosition() {
    return driveMotor.getSensorCollection().getIntegratedSensorVelocity();
  }

  @Override
  double getDriveTarget() {
    return optimizedState.speedMetersPerSecond;
  }

  @Override
  double getDriveMotorVoltage() {
    return driveMotor.getMotorOutputVoltage();
  }

  @Override
  double getDriveMotorOutputLevel() {
    return driveMotorOutput;
  }

  @Override
  double getDriveMotorFeedForward() {
    return driveMotorFeedforward;
  }

  @Override
  public boolean setStartPosition() {
    double target = dutyCycle.getOutput();

    // we spin the turn motor positive only, even if the alignment is behind us
    // expecting to loop around that align eventually. We may need to rethink this
    // approach if we have problems.
    if (Math.abs(target - wheelAlignment) > 2.0 * MAX_SLOP_FOR_WHEEL_ALIGNMNET) {
      turningMotor.set(0.2); // faster
    } else if (Math.abs(target - wheelAlignment) > MAX_SLOP_FOR_WHEEL_ALIGNMNET) {
      turningMotor.set(0.1); // slower
    } else {
      turningMotor.set(0.0); // stop
      turningEncoder.setPosition(0.0);
      driveMotor.setSelectedSensorPosition(0.0);
      return true;
    }

    return false;
  }

  @Override
  double getWheelAlignment() {
    return dutyCycle.getOutput();
  }
}
