// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve;


public class DiamondFormation extends Command {

  // private final Swerve mDrivebase;
  Swerve mDrivebase;
  /** Creates a new DiamondFormation. */
  public DiamondFormation(Swerve drivebase) {
    mDrivebase = drivebase;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // Prevents robot from budging by setting wheels 45 degrees inwards
    SwerveModuleState sms = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0));
    SwerveModuleState sms2 = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0));

    mDrivebase.setModuleStatesOverride(new SwerveModuleState[]{sms2, sms, sms, sms2});
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
