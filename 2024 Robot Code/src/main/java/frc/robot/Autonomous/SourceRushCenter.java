// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class SourceRushCenter extends SequentialCommandGroup {
        /** Creates a new SixPiece. */
        public SourceRushCenter(Swerve swerve, ShooterHood hood, Shooter shooter, Intake intake) {
                // Add your commands in the addCommands() call, e.g.
                addCommands(
                                // shoots preload after moving
                                new ParallelDeadlineGroup(new SequentialCommandGroup(
                                                swerve.followPathCommand("6 Piece B 0 Center",
                                                                true)

                                                                .withTimeout(4.),

                                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true,
                                                                false, 0)),
                                                new AlignToGoal(swerve).withTimeout(.75),

                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood,
                                                                () -> 28.5)),
                                                new RevUp(shooter, 1., .6, () -> 28.5, hood)),

                                new ParallelDeadlineGroup(
                                                new AutoIntake(intake, 1.),
                                                swerve.followPathCommand("6 Piece B 1 Center",
                                                                false))
                                                .withTimeout(3.),
                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false, 0)),

                                new ConditionalCommand(
                                                // drive back and shoot

                                                new SequentialCommandGroup(
                                                                new ParallelDeadlineGroup(new SequentialCommandGroup(
                                                                                swerve.followPathCommand(
                                                                                                "6 Piece B 2 Center",
                                                                                                false)
                                                                                                .withTimeout(4.),
                                                                                new InstantCommand(() -> swerve.drive(
                                                                                                new Translation2d(), 0,
                                                                                                true,
                                                                                                false, 0)),
                                                                                new AlignToGoalOnlyOdom(swerve)
                                                                                                .withTimeout(1.5),

                                                                                new ShootFalconForced(shooter, 1., 0.6,
                                                                                                intake, hood,
                                                                                                () -> 27)),
                                                                                new RevUp(shooter, 1., .6, () -> 27.,
                                                                                                hood)),

                                                                new ParallelDeadlineGroup(
                                                                                new AutoIntake(intake, 1.),
                                                                                swerve.followPathCommand(
                                                                                                "6 Piece B 3 Center",
                                                                                                false))
                                                                                .withTimeout(4.),
                                                                new InstantCommand(
                                                                                () -> swerve.drive(new Translation2d(),
                                                                                                0, true, false, 0)),
                                                                new DriveThenShoot(swerve, intake, shooter, hood,
                                                                                "6 Piece B 4 Center", 4., 25.)),
                                                // sliding to note then shooting
                                                new SequentialCommandGroup(
                                                                new DriveAndIntake(swerve, intake,
                                                                                "6 Piece B 2 Center B")
                                                                                .withTimeout(4.0),
                                                                new DriveThenShoot(swerve, intake, shooter, hood,
                                                                                "6 Piece B 4 Center", 4., 25.)),
                                                () -> intake.isInConveyor()),
                                new DriveAndIntake(swerve, intake, "6 Piece B 5 Center"), swerve.followPathCommand(
                                                "6 Piece B 6 Center", false));
        }
}
