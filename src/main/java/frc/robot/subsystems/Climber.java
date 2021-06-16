// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
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
  }

  @Override
  public void periodic() {
    
  }

  public void up() {
    leftMotor.set(0.2);
    rightMotor.set(0.2);
  }

  public void down() {
    leftMotor.set(-0.2);
    rightMotor.set(-0.2);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

}
