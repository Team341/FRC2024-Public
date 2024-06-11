// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class SixPieceSlide extends SequentialCommandGroup {
        /** Creates a new SixPiece. */
        public SixPieceSlide(Swerve swerve, ShooterHood hood, Shooter shooter, Intake intake) {
                // Add your commands in the addCommands() call, e.g.
                addCommands(
                                new FourPieceSlide(swerve, hood, shooter, intake),
                                new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(
                                                                new ParallelCommandGroup(
                                                                                new SequentialCommandGroup(
                                                                                                new WaitCommand(0.1),
                                                                                                new AutoIntake(intake,
                                                                                                                1.)),

                                                                                swerve.followChoreoPath(
                                                                                                "6Piece.3",
                                                                                                false)),
                                                                new InstantCommand(() -> swerve.drive(
                                                                                new Translation2d(), 0, true,
                                                                                false, 0)),
                                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood,
                                                                                () -> 25.5)),

                                                new RevUp(shooter, 1., 0.6, () -> 25.5,
                                                                hood)),

                                new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(
                                                                new ParallelCommandGroup(
                                                                                new SequentialCommandGroup(
                                                                                                new WaitCommand(0.75),
                                                                                                new AutoIntake(intake,
                                                                                                                1.)),

                                                                                swerve.followChoreoPath(
                                                                                                "6Piece.4",
                                                                                                false)),
                                                                new InstantCommand(() -> swerve.drive(
                                                                                new Translation2d(), 0, true,
                                                                                false, 0)),
                                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood,
                                                                                () -> 25.5)),

                                                new RevUp(shooter, 1., 0.6, () -> 25.5,
                                                                hood)),
                                new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(
                                                                new ParallelCommandGroup(
                                                                                new SequentialCommandGroup(
                                                                                                new WaitCommand(0.75),
                                                                                                new AutoIntake(intake,
                                                                                                                1.)),

                                                                                swerve.followChoreoPath(
                                                                                                "6Piece.5",
                                                                                                false)),
                                                                new InstantCommand(() -> swerve.drive(
                                                                                new Translation2d(), 0, true,
                                                                                false, 0)),
                                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood,
                                                                                () -> 35.)),

                                                new RevUp(shooter, 1., 0.6, () -> 35.,
                                                                hood))

                );
        }
}
