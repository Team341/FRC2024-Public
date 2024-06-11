// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
public class RushConditionalThree extends SequentialCommandGroup {
        /** Creates a new SixPiece. */
        public RushConditionalThree(Swerve swerve, ShooterHood hood, Shooter shooter, Intake intake) {
                // Add your commands in the addCommands() call, e.g.
                addCommands(
                                new ParallelDeadlineGroup(new SequentialCommandGroup(
                                                swerve.followPathCommand(
                                                                "Rush 1", true)
                                                                .withTimeout(2.5),
                                                new InstantCommand(
                                                                () -> swerve.drive(new Translation2d(),
                                                                                0, true, false, 0)),
                                                new AlignToGoal(swerve).withTimeout(0.5),

                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood,
                                                                () -> 22. -0.125)),
                                                new RevUp(shooter, 1., 0.6, () -> 22.-0.125,
                                                                hood)),
                                new ParallelDeadlineGroup(new SequentialCommandGroup(new WaitCommand(0.75),
                                                new AutoIntake(intake, 1.)),
                                                swerve.followPathCommand(
                                                                "Rush 2", false))
                                                .withTimeout(2.5),
                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false, 0)),
                                new ConditionalCommand(new SequentialCommandGroup(
                                                new ParallelDeadlineGroup(new SequentialCommandGroup(
                                                                swerve.followPathCommand(
                                                                                "Rush 3", false)
                                                                                .withTimeout(2.75),
                                                                new InstantCommand(() -> swerve.drive(
                                                                                new Translation2d(), 0, true,
                                                                                false, 0)),
                                                                new AlignToGoalOnlyOdom(swerve).withTimeout(0.5),

                                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood,
                                                                                () -> 26.-0.125)),
                                                                new RevUp(shooter, 1., .6, () -> 26.-0.125,    //bumped down .5
                                                                                hood)),
                                                new ParallelDeadlineGroup(
                                                                new SequentialCommandGroup(new WaitCommand(0.75),
                                                                                new AutoIntake(intake, 1.)),
                                                                swerve.followPathCommand(
                                                                                "Rush 4",
                                                                                false))
                                                                .withTimeout(2.75),
                                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true,
                                                                false, 0)),

                                                new ParallelDeadlineGroup(new SequentialCommandGroup(
                                                                swerve.followPathCommand(
                                                                                "Rush 5", false)
                                                                                .withTimeout(3.),
                                                                new InstantCommand(() -> swerve.drive(
                                                                                new Translation2d(), 0, true,
                                                                                false, 0)),
                                                                new AlignToGoalOnlyOdom(swerve).withTimeout(0.5),

                                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood,
                                                                                () -> 25.-0.125)),
                                                                new RevUp(shooter, 1., .6, () -> 25.-0.125,
                                                                                hood))),
                                                new SequentialCommandGroup(new ParallelDeadlineGroup(
                                                                new AutoIntake(intake, 1.),
                                                                swerve.followPathCommand(
                                                                                "Rush Alternative 1", false))
                                                                .withTimeout(2.5),
                                                                new ParallelDeadlineGroup(new SequentialCommandGroup(
                                                                                swerve.followPathCommand(
                                                                                                "Rush Alternative 2",
                                                                                                false)
                                                                                                .withTimeout(2.75),
                                                                                new InstantCommand(() -> swerve.drive(
                                                                                                new Translation2d(), 0,
                                                                                                true,
                                                                                                false, 0)),
                                                                                new AlignToGoalOnlyOdom(swerve).withTimeout(0.5),

                                                                                new ShootFalconForced(shooter, 1., 0.6,
                                                                                                intake, hood,
                                                                                                () -> 26.25)),
                                                                                new RevUp(shooter, 1., .6, () -> 26.25,
                                                                                                hood))),
                                                () -> intake.isInConveyor()),
                                new ParallelDeadlineGroup(new SequentialCommandGroup(new WaitCommand(0.75),
                                                new AutoIntake(intake, 1.)),
                                                swerve.followPathCommand(
                                                                "Rush 6",
                                                                false))
                                                .withTimeout(3.75),
                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false, 0)),
                                new ParallelDeadlineGroup(new SequentialCommandGroup(
                                                swerve.followPathCommand(
                                                                "Rush 7",
                                                                false)
                                                                .withTimeout(2.75),
                                                new InstantCommand(() -> swerve.drive(
                                                                new Translation2d(), 0,
                                                                true,
                                                                false, 0)),
                                                new AlignToGoalOnlyOdom(swerve).withTimeout(0.5),

                                                new ShootFalconForced(shooter, 1., 0.6,
                                                                intake, hood,
                                                                () -> 25.)),
                                                new RevUp(shooter, 1., .6, () -> 25.,
                                                                hood))

                );
        }
}
