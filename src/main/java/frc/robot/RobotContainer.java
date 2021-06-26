package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.teleop.DriveWithJoystick;
import frc.robot.commands.autonomous.DriveAndShootAuto;
import frc.robot.commands.autonomous.DriveDistance;
import frc.robot.commands.subsystem_controls.AlignWheelsCommand;
import frc.robot.subsystems.DrivetrainMax;
import frc.robot.subsystems.MotorTestSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.commands.Indexing.FeedBallToShooter;
import frc.robot.commands.Indexing.PickUpBall;
import frc.robot.subsystems.Feeder;

public class RobotContainer {
  // Subsystems
  public static Drivetrain s_drivetrain;
  private MotorTestSubsystem motorTestSubsystem;
  private Intake intake;
  private Climber climber;
  private Shooter shooter;
  private Indexer indexer;
  private Feeder feeder;

  private SendableChooser autonomousModeOption;

  // Driver Joysticks
  private Joystick l_joystick;
  private Joystick r_joystick;

  // Autonomous Components
  private DriveDistance driveDistanceAuto;

  // Autonomous Commands
  // private Command c_threeBall;
  // private Command c_eightBallVersionOne;
  // private Command c_eightBallVersionTwo;

  // Test Commands
  private boolean haveTestJoystick;
  private Joystick t_joystick;

  // Operator Joysticks
  private Joystick o_joystick;

  // Operator1Joystick Buttons

  public RobotContainer() {

    // Declare subsystems
    s_drivetrain = new Drivetrain();//DrivetrainMax();
    l_joystick = new Joystick(Constants.LEFT_JOYSTICK_PORT_ID);
    r_joystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT_ID);
    o_joystick = new Joystick(Constants.OPERATOR_JOYSTICK_PORT_ID);
    intake = new Intake();
    climber = new Climber();
    shooter = new Shooter();
    indexer = new Indexer();
    feeder = new Feeder();

    s_drivetrain.setDefaultCommand(new DriveWithJoystick(l_joystick, r_joystick, s_drivetrain, Boolean.valueOf(false)));

    // motorTestSubsystem = new MotorTestSubsystem(s_drivetrain);

    defineButtons(); // Define Buttons
    configureButtonBindings(); // Configure the button bindings
    defineAutonomousComponents(); // Define Autonomous Components
    reportJoystickData();
    initShuffleboardAuto();

  }

  private void defineButtons() {
    // Driver joystick declaration

    try {
      t_joystick = new Joystick(Constants.TEST_JOYSTICK_PORT_ID);
      if (t_joystick == null) {
        haveTestJoystick = false;
      } else {
        haveTestJoystick = true;
      }
    } catch (Exception e) {
      System.out.println("No test joystick found.  Test mode will not be available.");
      haveTestJoystick = false;
    }

    if (haveTestJoystick) {
      //new JoystickButton(t_joystick, Constants.L_JOYSTICKBUTTON_3).whenPressed(() -> motorTestSubsystem.runMotor(0.2))
      //    .whenReleased(() -> motorTestSubsystem.runMotor(0));
    }

  }

  private void configureButtonBindings() {
    new JoystickButton(l_joystick, Joystick.ButtonType.kTrigger.value).whileHeld(new PickUpBall(indexer, intake));

    new JoystickButton(l_joystick, Constants.R_JOYSTICKBUTTON_11).whenPressed(() -> climber.up(), climber)
        .whenReleased(() -> climber.stop(), climber);

    new JoystickButton(l_joystick, Constants.R_JOYSTICKBUTTON_10).whenPressed(() -> climber.down(), climber)
        .whenReleased(() -> climber.stop(), climber);

    new JoystickButton(l_joystick, Constants.R_JOYSTICKBUTTON_6).whenPressed(() -> climber.upfast(), climber)
        .whenReleased(() -> climber.stop(), climber);

    new JoystickButton(l_joystick, Constants.R_JOYSTICKBUTTON_7).whenPressed(() -> climber.downfast(), climber)
        .whenReleased(() -> climber.stop(), climber);

    new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_6).whenPressed(() -> shooter.maintainRPM())
        .whenReleased(() -> shooter.stopMotors());

    new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_4)
        .whileHeld(new FeedBallToShooter(indexer, intake, feeder));

    new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_5).whenPressed(() -> indexer.reverse())
        .whenReleased(() -> indexer.stop());

    // new JoystickButton(o_joystick,Constants.O_JOYSTICKBUTTON_12).whenPressed(() -> shooter.reverse())
    //     .whenReleased(() -> shooter.stop());

    // FIXME: pick a button to test wheel alignment
    // new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_2).whenPressed(new AlignWheelsCommand(s_drivetrain));

    // RightJoystick button configuration
    // r_button_2.whileHeld(new ArcadeDrive(s_drivetrain, r_joystick), true);
    // visionAlign.whenPressed(new DriveLimelight(drivetrain,
    // vision).withTimeout(1), true);

    // Operator joystick button configuration
    // enable climb do the shuffleboard
    // o_enableClimb.whenActive(new DisableSolenoids(climber));
    // o_enableClimb.whenInactive(new EnableSolenoids(climber));

  }

  private void reportJoystickData() {
    ShuffleboardTab tab = Shuffleboard.getTab("Joysticks");
    tab.addNumber("Left X-axis", l_joystick::getX);
    tab.addNumber("Left Y-axis", l_joystick::getY);
    tab.addNumber("Left Z-axis", l_joystick::getZ);
    tab.addBoolean("LB1", () -> l_joystick.getRawButton(Constants.L_JOYSTICKBUTTON_1));
    tab.addBoolean("LB2", () -> l_joystick.getRawButton(Constants.L_JOYSTICKBUTTON_2));
    tab.addBoolean("LB3", () -> l_joystick.getRawButton(Constants.L_JOYSTICKBUTTON_3));
    tab.addBoolean("LB4", () -> l_joystick.getRawButton(Constants.L_JOYSTICKBUTTON_4));
    tab.addBoolean("LB5", () -> l_joystick.getRawButton(Constants.L_JOYSTICKBUTTON_5));
    tab.addBoolean("LB6", () -> l_joystick.getRawButton(Constants.L_JOYSTICKBUTTON_6));
    tab.addBoolean("LB7", () -> l_joystick.getRawButton(Constants.L_JOYSTICKBUTTON_7));
    tab.addBoolean("LB8", () -> l_joystick.getRawButton(Constants.L_JOYSTICKBUTTON_8));
    tab.addBoolean("LB9", () -> l_joystick.getRawButton(Constants.L_JOYSTICKBUTTON_9));
    tab.addBoolean("LB10", () -> l_joystick.getRawButton(Constants.L_JOYSTICKBUTTON_10));

  }

  private void defineAutonomousComponents() {
    // driveDistanceAuto = new DriveDistance(s_drivetrain, -3);
  }

  private void defineAutonomousCommands() {
    // c_threeBall =
    // autoAlign.withTimeout(1).andThen(smartShooterAuto).withTimeout(5);

    // c_eightBallVersionOne =
    // autoAlign.withTimeout(1).andThen(smartShooterAuto).withTimeout(3).andThen(perpendicularAlign)
    // .andThen(autoIntakeOn).andThen(driveDistanceAuto).andThen(autoIntakeOff).andThen(autoAlign).withTimeout(1)
    // .andThen(smartShooterAuto).withTimeout(3);

    // c_eightBallVersionTwo =
    // autoIntakeOn.andThen(driveDistanceAuto).andThen(autoIntakeOff).andThen(autoAlign)
    // .withTimeout(1).andThen(smartShooterAuto).withTimeout(1).andThen(perpendicularAlign).andThen(autoIntakeOn)
    // .andThen(driveDistanceAuto).andThen(autoIntakeOff).andThen(driveDistanceAuto).andThen(autoAlign)
    // .andThen(smartShooterAuto).withTimeout(3);
  }

  private void initShuffleboardAuto() {

    // Good (working) example of Sendable Chooser to use dropdown option from
    // Shuffleboard to get a selected option
    // to run an auto routine

    // autonomousModeOption = new SendableChooser<>();
    // autonomousModeOption.setDefaultOption("Eight Ball Version 1",
    // c_eightBallVersionOne);
    // autonomousModeOption.addOption("Eight Ball Version 2",
    // c_eightBallVersionTwo);
    // autonomousModeOption.addOption("Drive Distance Auto", driveDistanceAuto);
    // SmartDashboard.putData("Auto selection", autonomousModeOption);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return (Command) autonomousModeOption.getSelected();
<<<<<<< Updated upstream
    return new DriveAndShootAuto(s_drivetrain, indexer, intake, feeder, shooter);
=======
    //return new DriveAndShootAuto(s_drivetrain, indexer, intake, feeder, shooter);
    return new InstantCommand(() -> intake.takeIn(true), intake)
    .andThen(new WaitCommand(0.5))
    .andThen(new InstantCommand(() -> intake.takeIn(false), intake))
    .andThen(new InstantCommand(() -> s_drivetrain.drive(-0.5d, 0d, 0, false), s_drivetrain))
    .andThen(new WaitCommand(3.5d))
    .andThen(new InstantCommand(() -> s_drivetrain.stop(), s_drivetrain))
    .andThen(new InstantCommand(() -> shooter.maintainRPM(), shooter))
    .andThen(new WaitCommand(3d))
    .andThen(new FeedBallToShooter(indexer, intake, feeder));
  }
  
  public void initTeleop(){
    s_drivetrain.stop();
    indexer.stop();
    intake.takeIn(false);
    feeder.stop();
    shooter.stopMotors();
>>>>>>> Stashed changes
  }

  public void setStartPosition() {
    //s_drivetrain.setStartPosition();
  }

}