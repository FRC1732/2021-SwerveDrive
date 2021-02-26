package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class HolonomicDriveCommand extends CommandBase {
    public HolonomicDriveCommand() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void execute() {
        boolean ignoreScalars = Robot.getOi().primaryController.getLeftBumperButton().get();

        double forward = Robot.getOi().primaryController.getLeftYAxis().get(true);
        double strafe = Robot.getOi().primaryController.getLeftXAxis().get(true);
        double rotation = Robot.getOi().primaryController.getRightXAxis().get(true, ignoreScalars);

        boolean robotOriented = Robot.getOi().primaryController.getXButton().get();
        boolean reverseRobotOriented = Robot.getOi().primaryController.getYButton().get();

        Vector2 translation = new Vector2(forward, strafe);

        if (reverseRobotOriented) {
            robotOriented = true;
            translation = translation.rotateBy(Rotation2.fromDegrees(180.0));
        }

        DrivetrainSubsystem.getInstance().holonomicDrive(translation, rotation, !robotOriented);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
