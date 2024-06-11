// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAndIntake extends ParallelDeadlineGroup {
  /** Creates a new DriveAndIntake. */
  public DriveAndIntake(Swerve swerve, Intake intake, String path) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new AutoIntake(intake, 1.),
        swerve.followPathCommand(
            path,
            false));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
