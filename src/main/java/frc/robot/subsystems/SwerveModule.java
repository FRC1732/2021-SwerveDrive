// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class SwerveModule extends PIDSubsystem {
  /** Creates a new SwerveModule. */

  TalonFX drive;
  CANSparkMax angle;
  double setpoint;

  /// ####################################################################
  /// THIS IS HERE AS AN EXAMPLE FOR BOTH TALON AND SPARK DECLARATIONS
  /// ####################################################################
  public SwerveModule(int talonID, int sparkID) {
    super(
        // The PIDController used by the subsystem
        new PIDController(1, 0, 0));
    drive = new TalonFX(talonID);
    angle = new CANSparkMax(sparkID, MotorType.kBrushless);

    
  }

  /// ####################################################################

  public void drive(double speed, double angle){
    drive.set(ControlMode.PercentOutput, speed);
    setpoint = angle * (12 * .5) + (12 * .5);
    if (setpoint < 0 ) setpoint = 12 + setpoint;
    if (setpoint > 12) setpoint = setpoint - 12;

  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
