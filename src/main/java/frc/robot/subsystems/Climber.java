// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  public Climber() {
    leftMotor = new CANSparkMax(Constants.CLIMBER_LEFT, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.CLIMBER_RIGHT, MotorType.kBrushless);
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    
  }

  public void upL() {
    leftMotor.set(0.15);
  }

  public void upR() {
    rightMotor.set(0.15);
  }

  public void downL() {
    leftMotor.set(-0.15);
  }

  public void downR() {
    rightMotor.set(-0.15);
  }

  public void upfastL() {
    leftMotor.set(0.45);
  }

  public void upfastR() {
    rightMotor.set(0.45);
  }

  public void downfastL() {
    leftMotor.set(-0.45);
  }

  public void downfastR() {
    rightMotor.set(-0.45);
  }

  public void stopL() {
    leftMotor.set(0);
  }

  public void stopR() {
    rightMotor.set(0);
  }

}