// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import frc.robot.Constants;

/** Add your docs here. */
public class Intake extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static final double kEncoderConversion = 1.0 * 2 * Math.PI / 53.3;
  private static final double motorSpeed = -0.7;

  private CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
  private CANEncoder intakeEncoder;
  
  public void Intake() {
    intakeEncoder = intakeMotor.getEncoder();
    intakeEncoder.setPositionConversionFactor(kEncoderConversion);
    intakeEncoder.setPosition(0);
  }

  @SuppressWarnings("ParameterName")
  public void takeIn(boolean run) {
    if (run)
      intakeMotor.set(motorSpeed);
    else
      intakeMotor.set(0);
  }

  public CANSparkMax getMotor() {
    return intakeMotor;
  }

}
