package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.commands.TeleOpDriveCommand;
import frc.robot.control.Controls;
import frc.robot.subsystems.DriveSubsystem;
import org.strykeforce.thirdcoast.telemetry.TelemetryController;
import org.strykeforce.thirdcoast.telemetry.TelemetryService;

public class RobotContainer {
  public static TelemetryService TELEMETRY;
  public static DriveSubsystem DRIVE;
  public static Controls CONTROLS;

  public RobotContainer() {

    if (RobotBase.isReal()) {
      TELEMETRY = new TelemetryService(TelemetryController::new);
      DRIVE = new DriveSubsystem();
      CONTROLS = new Controls();
      TELEMETRY.start();
      DRIVE.setDefaultCommand(new TeleOpDriveCommand());
    }
  }
}