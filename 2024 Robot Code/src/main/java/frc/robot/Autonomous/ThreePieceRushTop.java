// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drive.AlignToGoal;
import frc.robot.commands.Drive.AlignToGoalOnlyOdom;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.commands.Shooter.RevUp;
import frc.robot.commands.Shooter.ShootFalconForced;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreePieceRushTop extends SequentialCommandGroup {
        /** Creates a new SixPiece. */
        public ThreePieceRushTop(Swerve swerve, ShooterHood hood, Shooter shooter, Intake intake) {
                // Add your commands in the addCommands() call, e.g.
                addCommands(
                                new ParallelDeadlineGroup(new SequentialCommandGroup(
                                                swerve.followPathCommand(
                                                                "Rush Second 0", true)
                                                                .withTimeout(3.9),
                                                new InstantCommand(
                                                                () -> swerve.drive(new Translation2d(),
                                                                                0, true, false, 0)),
                                                new AlignToGoal(swerve).withTimeout(1.5),


                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood,
                                                                () -> 21.25)),
                                                new RevUp(shooter, 1., 0.6, () ->21.25,
                                                                hood)),
                                new ParallelDeadlineGroup(new SequentialCommandGroup(new WaitCommand(0.75),
                                                new AutoIntake(intake, 1.)),
                                                swerve.followPathCommand(
                                                                "Rush Second 1", false))
                                                .withTimeout(2.5),
                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false, 0)),
                                new ParallelDeadlineGroup(new SequentialCommandGroup(swerve.followPathCommand(
                                                "Rush 5", false)
                                                .withTimeout(2.75),
                                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true,
                                                                false, 0)),
                                                new AlignToGoalOnlyOdom(swerve).withTimeout(1.5),


                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood,
                                                                () -> 25.75)),
                                                new RevUp(shooter, 1., .6, () -> 25.75,
                                                                hood)),
                                new ParallelDeadlineGroup(new SequentialCommandGroup(new WaitCommand(0.75),
                                                new AutoIntake(intake, 1.)),
                                                swerve.followPathCommand(
                                                                "Rush 6",
                                                                false))
                                                .withTimeout(2.75),
                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false, 0)),

                                new ParallelDeadlineGroup(new SequentialCommandGroup(swerve.followPathCommand(
                                                "Rush 7", false)
                                                .withTimeout(3.),
                                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true,
                                                                false, 0)),
                                                new AlignToGoalOnlyOdom(swerve).withTimeout(1.5),


                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood,
                                                                () -> 24.5)),
                                                new RevUp(shooter, 1., .6, () -> 24.5,
                                                                hood))
                        
                );
        }
}
