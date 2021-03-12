package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.control.Controls;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  public static Drivetrain DRIVE;
  public static Controls CONTROLS;

  public RobotContainer() {

    if (RobotBase.isReal()) {
      DRIVE = new Drivetrain();
      CONTROLS = new Controls();
      DRIVE.setDefaultCommand(new DriveWithJoystick(null, null, null, null));
    }
  }
}