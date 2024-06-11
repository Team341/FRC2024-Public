// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.Swerve;

public class StrafeTilNote extends Command {
  Swerve mSwerve;
  double speed;
  
  /** Creates a new StrafeTilNote. */
  public StrafeTilNote(Swerve swerve) {
    mSwerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      speed = 1.5;
    }
    else speed = -1.5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while (LimelightInterface.getInstance().returnPieceData() == null) {
      mSwerve.drive(new Translation2d(0.0,speed), 0., isFinished(), isScheduled(), speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
