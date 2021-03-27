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

public class MotorTestSubsystem extends SubsystemBase {
  //private final TalonFX talonMotor;
  private final CANSparkMax neoMotor;

  /** Creates a new MotorTestSubsystem. */
  public MotorTestSubsystem() {
    //talonMotor = new TalonFX(Constants.DRIVETRAIN_BACK_LEFT_DRIVE);
    neoMotor = new CANSparkMax(Constants.DRIVETRAIN_FRONT_RIGHT_AZIMUTH, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotor(double speed) {
    //talonMotor.set(ControlMode.PercentOutput, speed);
    neoMotor.set(speed);
  }
}
