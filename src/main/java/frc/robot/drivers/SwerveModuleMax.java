// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import com.revrobotics.CANPIDController;

public class SwerveModuleMax extends AbstractSwerveModule {
  // Encoder returns revolutions; convert to radians; apply gear ratio
  private static final double TURN_ENCODER_CONVERSION = 1.0 * 2 * Math.PI / 53.3;
  // private static final double TURN_ENCODER_CONVERSION = 1.0 * 2 * Math.PI;

  private final TalonFX driveMotor;
  private final CANSparkMax turningMotor;
  private final CANEncoder turningEncoder;
  private final CANPIDController pidController;

  // no PID on drive, just direct control but will use a slew.
  // private final PIDController drivePIDController = new PIDController(1, 0, 0);
  private final ProfiledPIDController turningPIDController = new ProfiledPIDController(0.5, 0.01, 0.02,
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
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public double lastAngle;
  public double offset;

  private static final double TURNING_MOTOR_RATIO = 6.75422d;

  /**
   * Constructs a SwerveModuleMax.
   * 
   * @param talonID         ID for the drive motor.
   * @param sparkID         ID for the turning motor.
   * @param absoluteChannel Digital Input for absolute PWM signal
   * @param wheelAlignment  Value of wheel alignment when going forward
   * @param position        Notation of position of the module on the robot
   */
  public SwerveModuleMax(int talonID, int sparkID, int absoluteChannel, double wheelAlignment, SwervePosition position) {
    super(position);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.peakOutputForward = 1.0;
    talonConfig.peakOutputReverse = -1.0;
    talonConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    driveMotor = new TalonFX(talonID);
    driveMotor.configAllSettings(talonConfig, 100);
    driveMotor.setNeutralMode(NeutralMode.Coast);

    turningMotor = new CANSparkMax(sparkID, MotorType.kBrushless);
    turningMotor.restoreFactoryDefaults();
    pidController = turningMotor.getPIDController();
    turningEncoder = turningMotor.getEncoder();

    // PID coefficients
    kP = .5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    
    // turningEncoder.setPositionConversionFactor(TURN_ENCODER_CONVERSION);
    // turningEncoder.setPosition(0);

    // turningEncoder =
    // turningMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 1024);
    // turningEncoder.setPositionConversionFactor(TURN_ENCODER_CONVERSION);
    // turningEncoder.setPosition(0);

    optimizedState = new SwerveModuleState();
    //turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    //dutyCycle = new DutyCycle(new DigitalInput(absoluteChannel));
    this.wheelAlignment = wheelAlignment;

    offset = turningEncoder.getPosition();
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
   * Efficiently set the swerve module
   * 
   * @param degree Angle to set the wheel (any angle)
   * @param speed  Percent to set the wheel -1 to 1
   */
  private void setAbsolute(double targetDegree, double speed) {

    if (Math.abs(speed) < 0.1d) {
      driveMotor.set(ControlMode.PercentOutput, speed);
      return;
    }

    double currentEncoderActual = ((turningEncoder.getPosition() - offset) * TURNING_MOTOR_RATIO);

    double targetDegreeAbs = (targetDegree % 360 + 360) % 360;
    double currentEncoderAbs = (currentEncoderActual % 360 + 360) % 360;
    double delta = targetDegreeAbs - currentEncoderAbs;

    if (Math.abs(delta) > 180)
      delta = delta - (Math.signum(delta) * 360);

    if (Math.abs(delta) > 90){
      delta = delta - (Math.signum(delta) * 180);
      speed = speed * -1;
    }

    double newDegree = currentEncoderActual + delta;

    driveMotor.set(ControlMode.PercentOutput, speed);
    pidController.setReference((newDegree / TURNING_MOTOR_RATIO) + offset, ControlType.kPosition);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    
    // optimizedState = desiredState; // no optimization

    // driveMotorInput = convertMetersPerSecondToMotorVelocity(optimizedState.speedMetersPerSecond);
    // driveMotor.set(ControlMode.Velocity, driveMotorInput);
    

    //turnOutput = turningPIDController.calculate(turningEncoder.getPosition(), optimizedState.angle.getRadians());
    //turnFFVoltage = turnMotorFeedforward.calculate(turningPIDController.getSetpoint().velocity);

    //turningMotor.setVoltage(turnOutput + turnFFVoltage);

    // setAbsolute(desiredState.angle.getDegrees(), desiredState.speedMetersPerSecond / Constants.MAX_SPEED);
    if (Math.abs(desiredState.speedMetersPerSecond / Constants.MAX_SPEED) < 0.05d) {
      driveMotor.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond / Constants.MAX_SPEED);
      return;
    }

    double currentEncoderActual = ((turningEncoder.getPosition() - offset) * TURNING_MOTOR_RATIO);
    double currentEncoderAbs = (currentEncoderActual % 360 + 360) % 360;


    Rotation2d currentRotation = Rotation2d.fromDegrees((turningEncoder.getPosition() - offset) * TURNING_MOTOR_RATIO);
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);

    driveMotor.set(ControlMode.PercentOutput, (state.speedMetersPerSecond / Constants.MAX_SPEED) * 0.5d);
    pidController.setReference(((currentRotation.getDegrees() + rotationDelta.getDegrees()) / TURNING_MOTOR_RATIO) + offset, ControlType.kPosition);

  }

  @Override
  double getTurnPosition() {
    return turningEncoder.getPosition();
  }

  @Override
  double getTurnTarget() {
    return lastAngle;
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

  @Override
  CANPIDController getCANPIDController() {
    return turningMotor.getPIDController();
  }
}
