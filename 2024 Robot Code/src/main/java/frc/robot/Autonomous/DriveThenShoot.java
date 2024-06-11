// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drive.AlignToGoal;
import frc.robot.commands.Shooter.RevUp;
import frc.robot.commands.Shooter.ShootFalconForced;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveThenShoot extends SequentialCommandGroup {
  /** Creates a new DriveThenShoot. */
  public DriveThenShoot(Swerve swerve, Intake intake, Shooter shooter, ShooterHood hood, String path,
      double pathTimeout, double hoodAngle) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(new ParallelDeadlineGroup(new SequentialCommandGroup(
        swerve.followPathCommand(path,
            false)
            .withTimeout(pathTimeout),
        new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true,
            false, 0)),
        new AlignToGoal(swerve).withTimeout(1.5),

        new ShootFalconForced(shooter, 1., 0.6, intake, hood,
            () -> hoodAngle)),
        new RevUp(shooter, 1., .6, () -> hoodAngle, hood)));
  }
}
