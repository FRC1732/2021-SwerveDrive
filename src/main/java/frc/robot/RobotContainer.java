package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.commands.autonomous.DriveDistance;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  public static Drivetrain s_drivetrain;

  private SendableChooser autonomousModeOption;
  // Driver Joysticks
  private Joystick l_joystick;
  private Joystick r_joystick;

  // Autonomous Components
  private DriveDistance driveDistanceAuto;

  // Autonomous Commands
  private Command c_threeBall;
  private Command c_eightBallVersionOne;
  private Command c_eightBallVersionTwo;

  // LeftJoystick Buttons
  private JoystickButton l_button_1;
  private JoystickButton l_button_2;
  private JoystickButton l_button_3;
  private JoystickButton l_button_4;
  private JoystickButton l_button_5;
  private JoystickButton l_button_6;
  private JoystickButton l_button_7;
  private JoystickButton l_button_8;
  private JoystickButton l_button_9;
  private JoystickButton l_button_10;
  private JoystickButton l_button_11;

  // RightJoystick Buttons
  private JoystickButton r_button_1;
  private JoystickButton r_button_2;
  private JoystickButton r_button_3;
  private JoystickButton r_button_4;
  private JoystickButton r_button_5;
  private JoystickButton r_button_6;
  private JoystickButton r_button_7;
  private JoystickButton r_button_8;
  private JoystickButton r_button_9;
  private JoystickButton r_button_10;
  private JoystickButton r_button_11;

  // Operator Joysticks
  private Joystick o_joystick;

  // Operator1Joystick Buttons
  private JoystickButton o_button_1;
  private JoystickButton o_button_2;
  private JoystickButton o_button_3;
  private JoystickButton o_button_4;
  private JoystickButton o_button_5;
  private JoystickButton o_button_6;
  private JoystickButton o_button_7;
  private JoystickButton o_button_8;
  private JoystickButton o_button_9;
  private JoystickButton o_button_10;
  private JoystickButton o_button_11;
  private JoystickButton o_button_12;

  // Composed triggers
  private Trigger shoot;
  private Trigger climbUp;
  private Trigger climbDown;

  public RobotContainer() {

    // Declare subsystems
    s_drivetrain = new Drivetrain();
    s_drivetrain.setDefaultCommand(new DriveWithJoystick(null, null, null, null));

    // Define Buttons
    defineButtons();

    // Configure the button bindings
    configureButtonBindings();

    // Define Autonomous Components
    defineAutonomousComponents();

  }

  private void defineButtons() {
    // Driver joystick declaration
    l_joystick = new Joystick(Constants.LEFT_JOYSTICK_PORT_ID);
    r_joystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT_ID);

    // Operator joystick declaration
    o_joystick = new Joystick(Constants.OPERATOR_JOYSTICK_PORT_ID);

    // Driver - Left joystick button declaration
    l_button_1 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_1);
    l_button_2 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_2);
    l_button_3 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_3);
    l_button_4 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_4);
    l_button_5 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_5);
    l_button_6 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_6);
    l_button_7 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_7);
    l_button_8 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_8);
    l_button_9 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_9);
    l_button_10 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_10);
    l_button_11 = new JoystickButton(l_joystick, Constants.L_JOYSTICKBUTTON_11);

    // Driver - Right joystick button declaration
    r_button_1 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_1);
    r_button_2 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_2);
    r_button_3 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_3);
    r_button_4 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_4);
    r_button_5 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_5);
    r_button_6 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_6);
    r_button_7 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_7);
    r_button_8 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_8);
    r_button_9 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_9);
    r_button_10 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_10);
    r_button_11 = new JoystickButton(r_joystick, Constants.R_JOYSTICKBUTTON_11);

    // Operator - Joystick button declaration
    o_button_1 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_1);
    o_button_2 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_2);
    o_button_3 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_3);
    o_button_4 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_4);
    o_button_5 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_5);
    o_button_6 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_6);
    o_button_7 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_7);
    o_button_8 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_8);
    o_button_9 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_9);
    o_button_10 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_10);
    o_button_11 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_11);
    o_button_12 = new JoystickButton(o_joystick, Constants.O_JOYSTICKBUTTON_12);

    // Trigger declaration examples
    // shoot = o_maintainRPM.and(smartShooter);
    // climbUp = o_enableClimb.and(o_manualUp);
    // climbDown = o_enableClimb.and(o_manualDown);

  }

  private void configureButtonBindings() {
    // leftJoystick button configuration
    // l_button_1.whileHeld(new IntakeCells(intake));
    // l_button_2.whenPressed(new ToggleIntakeSolenoidState(intake));

    // RightJoystick button configuration
    // r_button_2.whileHeld(new ArcadeDrive(s_drivetrain, r_joystick), true);
    // visionAlign.whenPressed(new DriveLimelight(drivetrain,
    // vision).withTimeout(1), true);

    // Operator joystick button configuration
    // enable climb do the shuffleboard
    // o_enableClimb.whenActive(new DisableSolenoids(climber));
    // o_enableClimb.whenInactive(new EnableSolenoids(climber));
    // o_manualSpeedUp.whenPressed(new ShooterManualUp(shooter));
    // o_manualSpeedDown.whenPressed(new ShooterManualDown(shooter));
    // o_maintainRPM.whenActive(new MaintainRPM(shooter, operatorJoystick));
    // o_maintainRPM.whenInactive(new StopMotors(shooter));
    // o_unallocatedButton.whenActive(new ToggleLimelightLEDS(vision));
    // o_unallocatedButton.whenInactive(new ToggleLimelightLEDS(vision));
    // o_changeIntakeSolenoidState.whenActive(new
    // SetIntakeSolenoidExtended(intake));
    // o_changeIntakeSolenoidState.whenInactive(new
    // SetIntakeSolenoidRetracted(intake));
    // o_rotateToLimelight.whileHeld(new AutoAlign(drivetrain, vision),true);
    // o_reverseIntake.whenHeld(new ReverseIntakeCells(intake));
    // o_feedShooter.whenHeld(new FeedShooter(indexer));
    // o_reverseFeedShooter.whenHeld(new ReverseFeedShooter(indexer));

    // Trigger declaration
    // shoot.whenActive(new SmartShooter(indexer, shooter));
    // shoot.whenInactive(new StopSmartShooter(shooter, indexer));
    // climbUp.whenActive(new ManualDown(climber));
    // climbUp.whenInactive(new StopClimber(climber));
    // climbDown.whenActive(new ManualUp(climber));
    // climbDown.whenInactive(new StopClimber(climber));
  }

  private void defineAutonomousComponents() {
    driveDistanceAuto = new DriveDistance(s_drivetrain, -3);
  }

  private void defineAutonomousCommands() {
    // c_threeBall = autoAlign.withTimeout(1).andThen(smartShooterAuto).withTimeout(5);

    // c_eightBallVersionOne = autoAlign.withTimeout(1).andThen(smartShooterAuto).withTimeout(3).andThen(perpendicularAlign)
    //     .andThen(autoIntakeOn).andThen(driveDistanceAuto).andThen(autoIntakeOff).andThen(autoAlign).withTimeout(1)
    //     .andThen(smartShooterAuto).withTimeout(3);

    // c_eightBallVersionTwo = autoIntakeOn.andThen(driveDistanceAuto).andThen(autoIntakeOff).andThen(autoAlign)
    //     .withTimeout(1).andThen(smartShooterAuto).withTimeout(1).andThen(perpendicularAlign).andThen(autoIntakeOn)
    //     .andThen(driveDistanceAuto).andThen(autoIntakeOff).andThen(driveDistanceAuto).andThen(autoAlign)
    //     .andThen(smartShooterAuto).withTimeout(3);
  }

  private void initShuffleboard() {
    autonomousModeOption = new SendableChooser<>();
    autonomousModeOption.setDefaultOption("Eight Ball Version 1", c_eightBallVersionOne);
    autonomousModeOption.addOption("Eight Ball Version 2", c_eightBallVersionTwo);
    autonomousModeOption.addOption("Drive Distance Auto", driveDistanceAuto);
    SmartDashboard.putData("Auto selection", autonomousModeOption);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return (Command) autonomousModeOption.getSelected();
  }

}