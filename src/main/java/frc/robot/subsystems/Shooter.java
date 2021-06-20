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
  private static final int SETPOINT = 10000;
  private static final int DEADBAND = 1000;

  public Shooter() {

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.peakOutputForward = 1.0;
    talonConfig.peakOutputReverse = 1.0;
    talonConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    motorOne = new TalonFX(Constants.MOTORMASTER);
    motorOne.configAllSettings(talonConfig, 100);
    motorOne.setNeutralMode(NeutralMode.Coast);
    motorTwo.setInverted(true);

    motorTwo = new TalonFX(Constants.MOTORFOLLOWER);
    motorTwo.configAllSettings(talonConfig, 100);
    motorTwo.setNeutralMode(NeutralMode.Coast);
    motorTwo.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    if (motorOne.getSelectedSensorVelocity() > 100){
    System.out.println("Speed| "+ motorOne.getSelectedSensorVelocity());
    System.out.println("Voltage| "+ motorOne.getBusVoltage());}
  }

  public void testMotors(){
    motorOne.set(ControlMode.PercentOutput, 0.75d);
    motorTwo.set(ControlMode.PercentOutput, 0.75d);
  }

  public boolean maintainRPM() {
    if(motorOne.getSelectedSensorVelocity() < SETPOINT){
      motorOne.set(ControlMode.PercentOutput, 0.9d);
      motorTwo.set(ControlMode.PercentOutput, 0.9d);
    } else {
      motorOne.set(ControlMode.PercentOutput, .5d);
      motorTwo.set(ControlMode.PercentOutput, .5d);
    }
    return motorOne.getSelectedSensorVelocity() > SETPOINT - DEADBAND;
  }

  public void stopMotors(){
    motorOne.set(ControlMode.PercentOutput, 0);
  }
}


