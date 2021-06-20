// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;

public class PickUpBall extends CommandBase {
  private Indexer indexer;
  private Intake intake;
  private Feeder feeder;

  /** Creates a new PickUpBall. */
  public PickUpBall(Indexer indexer, Intake intake, Feeder feeder) {
    addRequirements(indexer);
    addRequirements(intake);
    this.indexer = indexer;
    this.intake = intake;
    this.feeder = feeder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.index();
    intake.takeIn(true);
    feeder.feed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
    intake.takeIn(false);
    feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
