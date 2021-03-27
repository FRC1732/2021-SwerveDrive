// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.autonomous.DriveDistance;
import frc.robot.drivers.SwerveModule;
import frc.robot.drivers.SwervePosition;

public class MotorTestSubsystem extends SubsystemBase {
  private final TalonFX frTalonMotor;
  private final TalonFX flTalonMotor;
  private final TalonFX brTalonMotor;
  private final TalonFX blTalonMotor;
  private final CANSparkMax frNeoMotor;
  private final CANSparkMax flNeoMotor;
  private final CANSparkMax brNeoMotor;
  private final CANSparkMax blNeoMotor;

  /** Creates a new MotorTestSubsystem. */
  public MotorTestSubsystem(Drivetrain drivetrain) {
    SwerveModule swerveModule;
    // FR
    swerveModule = drivetrain.getByPosition(SwervePosition.FrontRight);
    frTalonMotor = swerveModule.getDriveMotor();
    frNeoMotor = swerveModule.getTurningMotor();
    // FL
    swerveModule = drivetrain.getByPosition(SwervePosition.FrontLeft);
    flTalonMotor = swerveModule.getDriveMotor();
    flNeoMotor = swerveModule.getTurningMotor();
    // BR
    swerveModule = drivetrain.getByPosition(SwervePosition.BackRight);
    brTalonMotor = swerveModule.getDriveMotor();
    brNeoMotor = swerveModule.getTurningMotor();
    // BL
    swerveModule = drivetrain.getByPosition(SwervePosition.BackLeft);
    blTalonMotor = swerveModule.getDriveMotor();
    blNeoMotor = swerveModule.getTurningMotor();      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotor(double speed) {
    //talonMotor.set(ControlMode.PercentOutput, speed);
    //neoMotor.set(speed);
  }
}
