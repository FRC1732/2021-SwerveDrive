// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private TalonFX motorOne;
  private TalonFX motorTwo;
  private double motorPercentage;

  public Shooter() {

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.peakOutputForward = 1.0;
    talonConfig.peakOutputReverse = 1.0;
    talonConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    motorOne = new TalonFX(Constants.MOTORMASTER);
    motorOne.configAllSettings(talonConfig, 100);
    motorOne.setNeutralMode(NeutralMode.Coast);

    motorTwo = new TalonFX(Constants.MOTORFOLLOWER);
    motorTwo.configAllSettings(talonConfig, 100);
    motorTwo.setNeutralMode(NeutralMode.Coast);
    motorTwo.setInverted(true);
    motorTwo.follow(motorOne);

    motorPercentage = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorOne.set(ControlMode.PercentOutput, motorPercentage);
  }

  public void shooterOn(){
    motorPercentage = .8;
  }

  public void reverse(){
    motorPercentage = -.1;
  }

  public void stop(){
    motorPercentage = 0;
  }

  public void increaseSpeed(){
    motorPercentage += .01;
  }

  public void decreaseSpeed(){
    motorPercentage -= .01;
  }
  

}


