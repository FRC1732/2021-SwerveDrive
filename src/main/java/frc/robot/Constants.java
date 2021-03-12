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
  
    //================== Subsystem ::: Drivetrain ==================
    public static final int DRIVETRAIN_M1_AZIMUTH = 0;
    public static final int DRIVETRAIN_M2_AZIMUTH = 1;
    public static final int DRIVETRAIN_M3_AZIMUTH = 2;
    public static final int DRIVETRAIN_M4_AZIMUTH = 3;
     
    public static final int DRIVETRAIN_M1_DRIVE = 10;
    public static final int DRIVETRAIN_M2_DRIVE = 11;
    public static final int DRIVETRAIN_M3_DRIVE = 12;
    public static final int DRIVETRAIN_M4_DRIVE = 13;

    //================== Driver Interface (Joysticks) ==================
    public static final int LEFT_JOYSTICK_PORT_ID = 0;
    public static final int RIGHT_JOYSTICK_PORT_ID = 1;

    // ================== Operator Interface (buttons) ==================
    public static final int OPERATOR_JOYSTICK_PORT_ID = 2;
    public static final int O_JOYSTICKBUTTON_ENABLE_CLIMB = 1;
    public static final int O_JOYSTICKBUTTON_MANUAL_UP = 2;
    public static final int O_JOYSTICKBUTTON_MANUAL_DOWN = 3;
    public static final int O_JOYSTICKBUTTON_MANUAL_SPEED_UP = 4;
    public static final int O_JOYSTICKBUTTON_MANUAL_SPEED_DOWN = 5;
    public static final int O_JOYSTICKBUTTON_MAINTAIN_RPM = 6;
    public static final int O_JOYSTICKBUTTON_UNALLOCATED_BUTTON = 7;
    public static final int O_JOYSTICKBUTTON_CHANGE_INTAKE_SOLENOID_STATE = 8;
    public static final int O_JOYSTICKBUTTON_ROTATE_TO_LIMELIGHT = 9;
    public static final int O_JOYSTICKBUTTON_REVERSE_INTAKE = 10;
    public static final int O_JOYSTICKBUTTON_FEED_SHOOTER = 11;
    public static final int O_JOYSTICKBUTTON_REVERSE_FEED_SHOOTER = 12;

}

