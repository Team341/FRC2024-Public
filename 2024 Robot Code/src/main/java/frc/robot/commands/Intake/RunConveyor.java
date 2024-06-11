// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunConveyor extends Command {
  /** Creates a new RunConveyor. */
  Intake intake;
  double speed;
  public RunConveyor(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);          

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  boolean done = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setSpeedConveyor(speed);
    if (intake.isInConveyor()) {
      intake.setSpeedConveyor(-1.);
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeedConveyor(0.);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
