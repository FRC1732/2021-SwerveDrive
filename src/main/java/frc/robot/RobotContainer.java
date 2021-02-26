package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.control.Controls;
import frc.robot.subsystems.TBSwerve_DriveSubsystem;
import org.strykeforce.thirdcoast.telemetry.TelemetryController;
import org.strykeforce.thirdcoast.telemetry.TelemetryService;

public class RobotContainer {
  public static TelemetryService TELEMETRY;
  public static TBSwerve_DriveSubsystem DRIVE;
  public static Controls CONTROLS;

  public RobotContainer() {

    if (RobotBase.isReal()) {

      TELEMETRY = new TelemetryService(TelemetryController::new);

      DRIVE = new TBSwerve_DriveSubsystem();
      CONTROLS = new Controls();

      TELEMETRY.start();

      DRIVE.setDefaultCommand(new TeleOpDriveCommand());
    }
  }
}