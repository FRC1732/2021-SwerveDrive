// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

  private CANSparkMax feedMotor;
  /** Creates a new Feeder. */
  public Feeder() {
    feedMotor = new CANSparkMax(Constants.STAGING_WHEEL, MotorType.kBrushed);
  }


  public void feed() {
    feedMotor.set(Constants.FEEDER_MOTOR_SPEED);
  }

  public void stop() {
      feedMotor.set(0);
  }
}
