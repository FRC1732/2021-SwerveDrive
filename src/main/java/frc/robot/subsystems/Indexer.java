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

public class Indexer extends SubsystemBase {

  private TalonFX brushMotor;
  /** Creates a new Indexer. */
  public Indexer() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.peakOutputForward = 1.0;
    talonConfig.peakOutputReverse = -1.0;
    talonConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    brushMotor = new TalonFX(Constants.BRUSH_WHEEL);
    brushMotor.configAllSettings(talonConfig, 100);
    brushMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void index() {
    brushMotor.set(ControlMode.PercentOutput, 0.4);
  }

  public void reverse() {
    brushMotor.set(ControlMode.PercentOutput, -0.25);
  }

  public void stop() {
      brushMotor.set(ControlMode.PercentOutput, 0);
  }
}
