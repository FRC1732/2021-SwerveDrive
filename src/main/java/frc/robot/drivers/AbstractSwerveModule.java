// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivers;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

/** Add your docs here. */
public abstract class AbstractSwerveModule {
    private final SwervePosition swervePosition;
    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve Modules");

    public AbstractSwerveModule(SwervePosition swervePosition) {
        this.swervePosition = swervePosition;

        initShuffleBoard();
    }

    private void initShuffleBoard() {
        ShuffleboardLayout layout = tab.getLayout(swervePosition.toString(), BuiltInLayouts.kList).withSize(2, 5);

        layout.addNumber("Turn Position (r)", this::getTurnPosition);
        layout.addNumber("Turn Target (r)", this::getTurnTarget);
        layout.addNumber("Turn Motor Voltage (V)", this::getTurnMotorVoltage);
        layout.addNumber("Turn Motor Output (V)", this::getTurnMotorOutputLevel);
        layout.addNumber("Turn Motor FF (V)", this::getTurnMotorFeedForward);

        // layout.addNumber("Drive Position (m-s)", this::getDrivePosition);
        // layout.addNumber("Drive Target (m-s)", this::getDriveTarget);
        // layout.addNumber("Drive Motor Voltage (V)", this::getDriveMotorVoltage);
        // layout.addNumber("Drive Motor Output (m-s)", this::getDriveMotorOutputLevel);
        // layout.addNumber("Drive Motor FF (m-s)", this::getDriveMotorFeedForward);
    }

    protected double convertMotorVelocityToMetersSecond(double motorVelocity) {
        // guessing at a max RPM to equate to max speed; really this is mechanical math,
        // a wheel turning to an encoder result in a 100ms period
        // 6000 RPM --> 4.0 m/s
        // RPM * encoder resolution / number of 100ms in a minute
        double maxVelocityUnitsPer100ms = 6000.0 * 2048.0 / 600.0;
        return motorVelocity * Constants.MAX_SPEED / maxVelocityUnitsPer100ms;
    }

    protected double convertMetersPerSecondToMotorVelocity(double metersPerSecond) {
        // guessing at a max RPM to equate to max speed; really this is mechanical math,
        // a wheel turning to an encoder result in a 100ms period
        // 6000 RPM --> 4.0 m/s
        // RPM * encoder resolution / number of 100ms in a minute
        double maxVelocityUnitsPer100ms = 6000.0 * 2048.0 / 600.0;
        return metersPerSecond * maxVelocityUnitsPer100ms / Constants.MAX_SPEED;
    }


    abstract double getTurnPosition();

    abstract double getTurnTarget();

    abstract double getTurnMotorVoltage();

    abstract double getTurnMotorOutputLevel();

    abstract double getTurnMotorFeedForward();

    abstract double getDrivePosition();

    abstract double getDriveTarget();

    abstract double getDriveMotorVoltage();

    abstract double getDriveMotorOutputLevel();

    abstract double getDriveMotorFeedForward();
}
