// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;


public class RunIntake extends Command {
  /** Creates a new ShootFalcon. */
  Intake intake;
  public RunIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies
  }

  @Override
  public void initialize() {
    intake.setSpeedIntake(Constants.Intake.INTAKE_SPEED);
    intake.deploy();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setSpeedIntake(-Constants.Intake.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeedIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isInIntake();
  
}}
