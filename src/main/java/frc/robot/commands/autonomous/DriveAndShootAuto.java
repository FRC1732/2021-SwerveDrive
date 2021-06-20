// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Indexing.FeedBallToShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class DriveAndShootAuto extends CommandBase {

  private Drivetrain drivetrain;
  private Indexer indexer;
  private Intake intake;
  private Feeder feeder;
  private Shooter shooter;

  /** Creates a new DriveAndShootAuto. */
  public DriveAndShootAuto(Drivetrain drivetrain, Indexer indexer, Intake intake, Feeder feeder, Shooter shooter) {
    this.drivetrain = drivetrain;
    this.indexer = indexer;
    this.intake = intake;
    this.feeder = feeder;
    this.shooter = shooter;

    new InstantCommand(() -> intake.takeIn(true), intake)
    .andThen(new WaitCommand(0.25))
    .andThen(new InstantCommand(() -> intake.takeIn(false), intake))
    .andThen(new InstantCommand(() -> drivetrain.drive(-0.25d, 0d, 0d, false), drivetrain))
    .andThen(new WaitCommand(4d))
    .andThen(new InstantCommand(() -> drivetrain.stop(), drivetrain))
    .andThen(new InstantCommand(() -> shooter.maintainRPM(), shooter))
    .andThen(new WaitCommand(4d))
    .andThen(new FeedBallToShooter(indexer, intake, feeder));
    //drop intake
    //move forward 1/2 second
    //move right 


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    indexer.stop();
    intake.takeIn(false);
    feeder.stop();
    shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
