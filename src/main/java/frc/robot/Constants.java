/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // ================== Subsystem ::: Drivetrain ==================
    public static final int DRIVETRAIN_BACK_RIGHT_AZIMUTH = 16;
    public static final int DRIVETRAIN_FRONT_RIGHT_AZIMUTH = 17;
    public static final int DRIVETRAIN_FRONT_LEFT_AZIMUTH = 19;
    public static final int DRIVETRAIN_BACK_LEFT_AZIMUTH = 18;

    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE = 2;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE = 3;
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE = 4;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE = 5;

    public static final int DRIVETRAIN_BACK_RIGHT_ALIGNMENT_CHANNEL = 9;
    public static final int DRIVETRAIN_FRONT_RIGHT_ALIGNMENT_CHANNEL = 7;
    public static final int DRIVETRAIN_FRONT_LEFT_ALIGNMENT_CHANNEL = 6;
    public static final int DRIVETRAIN_BACK_LEFT_ALIGNMENT_CHANNEL = 8;

    public static final double DRIVETRAIN_BACK_RIGHT_ALIGNMENT_TARGET = 0.633;
    public static final double DRIVETRAIN_FRONT_RIGHT_ALIGNMENT_TARGET = 0.812;
    public static final double DRIVETRAIN_FRONT_LEFT_ALIGNMENT_TARGET = 0.120;
    public static final double DRIVETRAIN_BACK_LEFT_ALIGNMENT_TARGET = 0.204;

    // ================== Subsystem ::: Intake ==================
    public static final int INTAKE = 23;
    public static final double MOTOR_SPEED = -0.7;

    // ================== Subsystem ::: Climber ==================
    public static final int CLIMBER_LEFT = 25;
    public static final int CLIMBER_RIGHT = 26;

    // ================== Driver ::: Left Joystick ::: Mapping ==================
    public static final int LEFT_JOYSTICK_PORT_ID = 0;
    public static final int L_JOYSTICKBUTTON_1 = 1;
    public static final int L_JOYSTICKBUTTON_2 = 2;
    public static final int L_JOYSTICKBUTTON_3 = 3;
    public static final int L_JOYSTICKBUTTON_4 = 4;
    public static final int L_JOYSTICKBUTTON_5 = 5;
    public static final int L_JOYSTICKBUTTON_6 = 6;
    public static final int L_JOYSTICKBUTTON_7 = 7;
    public static final int L_JOYSTICKBUTTON_8 = 8;
    public static final int L_JOYSTICKBUTTON_9 = 9;
    public static final int L_JOYSTICKBUTTON_10 = 10;
    public static final int L_JOYSTICKBUTTON_11 = 11;
    public static final int L_JOYSTICKBUTTON_12 = 12;

    // ================== Driver ::: Right Joystick ::: Mapping ==================
    public static final int RIGHT_JOYSTICK_PORT_ID = 1;
    public static final int R_JOYSTICKBUTTON_1 = 1;
    public static final int R_JOYSTICKBUTTON_2 = 2;
    public static final int R_JOYSTICKBUTTON_3 = 3;
    public static final int R_JOYSTICKBUTTON_4 = 4;
    public static final int R_JOYSTICKBUTTON_5 = 5;
    public static final int R_JOYSTICKBUTTON_6 = 6;
    public static final int R_JOYSTICKBUTTON_7 = 7;
    public static final int R_JOYSTICKBUTTON_8 = 8;
    public static final int R_JOYSTICKBUTTON_9 = 9;
    public static final int R_JOYSTICKBUTTON_10 = 10;
    public static final int R_JOYSTICKBUTTON_11 = 11;
    public static final int R_JOYSTICKBUTTON_12 = 12;

    // ================== Operator ::: Joystick ::: Mapping ==================
    public static final int OPERATOR_JOYSTICK_PORT_ID = 2;
    public static final int O_JOYSTICKBUTTON_1 = 1;
    public static final int O_JOYSTICKBUTTON_2 = 2;
    public static final int O_JOYSTICKBUTTON_3 = 3;
    public static final int O_JOYSTICKBUTTON_4 = 4;
    public static final int O_JOYSTICKBUTTON_5 = 5;
    public static final int O_JOYSTICKBUTTON_6 = 6;
    public static final int O_JOYSTICKBUTTON_7 = 7;
    public static final int O_JOYSTICKBUTTON_8 = 8;
    public static final int O_JOYSTICKBUTTON_9 = 9;
    public static final int O_JOYSTICKBUTTON_10 = 10;
    public static final int O_JOYSTICKBUTTON_11 = 11;
    public static final int O_JOYSTICKBUTTON_12 = 12;

    // ================== Test ::: Joystick ::: Mapping ==================
    public static final int TEST_JOYSTICK_PORT_ID = 3;
    public static final int T_JOYSTICKBUTTON_1 = 1;
    public static final int T_JOYSTICKBUTTON_2 = 2;
    public static final int T_JOYSTICKBUTTON_3 = 3;
    public static final int T_JOYSTICKBUTTON_4 = 4;
    public static final int T_JOYSTICKBUTTON_5 = 5;
    public static final int T_JOYSTICKBUTTON_6 = 6;
    public static final int T_JOYSTICKBUTTON_7 = 7;
    public static final int T_JOYSTICKBUTTON_8 = 8;
    public static final int T_JOYSTICKBUTTON_9 = 9;
    public static final int T_JOYSTICKBUTTON_10 = 10;
    public static final int T_JOYSTICKBUTTON_11 = 11;
    public static final int T_JOYSTICKBUTTON_12 = 12;

    public static final double MAX_SPEED = 4.0; // 4.0 meters per second (under the theoretical max of 15.9 ft/s
    // provided by mechanical)

    // putting in a 10x to make the modules turn faster. How fast can we go?
    public static final double MAX_ANGULAR_VELOCITY = Math.PI * 3; // radians per second
    public static final double MAX_ANGULAR_ACCELERATION = 10.0 * 2 * Math.PI; // radians per second squared
    public static final boolean SWERVE_USE_ALTERNATE_ENCODER = false;

}
