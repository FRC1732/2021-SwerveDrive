// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

/** Add your docs here. */
public class Intake extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // private static final double kEncoderConversion = 1.0 * 2 * Math.PI / 53.3;

  private CANSparkMax intakeMotor;
  // private CANEncoder intakeEncoder;

  public Intake() {
    intakeMotor = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
  }

  public void takeIn(boolean run) {
    if (run)
      intakeMotor.set(Constants.MOTOR_SPEED);
    else
      intakeMotor.set(0);
    // System.out.println("run");
  }

  public CANSparkMax getMotor() {
    return intakeMotor;
  }

  public void reverse() {
    intakeMotor.set(Constants.MOTOR_SPEED / -2.0); // half and reserse of normal
  }

  public void stop() {
    intakeMotor.set(0.0);
  }

}
