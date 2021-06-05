// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivers;

import java.util.Map;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import frc.robot.Constants;

/** Add your docs here. */
public abstract class AbstractSwerveModule {
    private final SwervePosition swervePosition;
    private ShuffleboardTab tab = Shuffleboard.getTab("Swerve Modules");
    private ShuffleboardTab tabPID = Shuffleboard.getTab("Swerve Modules - PID");

    // guessing at a max RPM to equate to max speed; really this is mechanical math,
    // a wheel turning to an encoder result in a 100ms period
    // 6000 RPM --> 4.0 m/s
    // RPM * encoder resolution / number of 100ms in a minute
    protected final double MAX_VELOCITY_PER_100MS = 6000.0 * 2048.0 / 600.0;

    // Max slop allowed for wheel alignment
    protected final double MAX_SLOP_FOR_WHEEL_ALIGNMNET = 0.1;

    public AbstractSwerveModule(SwervePosition swervePosition) {
        this.swervePosition = swervePosition;

        initShuffleBoard();
    }

    private void initShuffleBoard() {
        ShuffleboardLayout layout = tab.getLayout(swervePosition.toString(), BuiltInLayouts.kList).withSize(2, 4);

        layout.addNumber("Turn Position (r)", this::getTurnPosition);
        layout.addNumber("Turn Target (r)", this::getTurnTarget);
        layout.addNumber("Turn Motor Voltage (V)", this::getTurnMotorVoltage);
        layout.addNumber("Turn Motor Output (V)", this::getTurnMotorOutputLevel);
        layout.addNumber("Turn Motor FF (V)", this::getTurnMotorFeedForward);

        layout.addNumber("Drive Position (m-s)", this::getDrivePosition);
        layout.addNumber("Drive Target (m-s)", this::getDriveTarget);
        layout.addNumber("Drive Motor Voltage (V)", this::getDriveMotorVoltage);
        layout.addNumber("Drive Motor Output (m-s)", this::getDriveMotorOutputLevel);
        layout.addNumber("Drive Motor FF (m-s)", this::getDriveMotorFeedForward);

        layout.addNumber("Wheel Alignment", this::getWheelAlignment);

        ShuffleboardLayout layoutPid = tabPID.getLayout(swervePosition.toString(), BuiltInLayouts.kList).withSize(2, 4);

        layoutPid.addNumber("Turn Motor Voltage", () -> getTurnMotorVoltage()).withWidget(BuiltInWidgets.kNumberBar);
        layoutPid.addNumber("P", ()-> getCANPIDController().getP());
        layoutPid.addNumber("I", ()-> getCANPIDController().getI());
        layoutPid.addNumber("D", ()-> getCANPIDController().getD());
        layoutPid.addNumber("FF", ()-> getCANPIDController().getFF());
        layoutPid.addNumber("Max Output", ()-> getCANPIDController().getOutputMax());
        layoutPid.addNumber("Min Output", ()-> getCANPIDController().getOutputMin());
    }

    protected double convertMotorVelocityToMetersSecond(double motorVelocity) {
        return motorVelocity * Constants.MAX_SPEED / MAX_VELOCITY_PER_100MS;
    }

    protected double convertMetersPerSecondToMotorVelocity(double metersPerSecond) {
        return metersPerSecond * MAX_VELOCITY_PER_100MS / Constants.MAX_SPEED;
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

    abstract double getWheelAlignment();

    abstract boolean setStartPosition();

    abstract CANPIDController getCANPIDController();
}
