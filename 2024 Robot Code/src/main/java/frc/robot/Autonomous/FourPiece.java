// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Drive.AlignToGoalOnlyOdom;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.commands.Shooter.RevUp;
import frc.robot.commands.Shooter.RevUpFirstShot;
import frc.robot.commands.Shooter.ShootFalcon;
import frc.robot.commands.Shooter.ShootFalconForced;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourPiece extends SequentialCommandGroup {
        /** Creates a new SixPiece. */

        // Assuming this is a method in your drive subsystem

        public FourPiece(Swerve swerve, ShooterHood hood, Shooter shooter, Intake intake) {
                // Add your commands in the addCommands() call, e.g.
                addCommands(
                                // shoot preload
                                new ParallelDeadlineGroup(
                                                new ShootFalcon(shooter, 1., 1.0, intake, hood,
                                                                () -> Constants.Shooter.SPEAKER_CLOSE_ANGLE),
                                                new RevUpFirstShot(shooter, .85, .85,
                                                                () -> Constants.Shooter.SPEAKER_CLOSE_ANGLE,
                                                                hood)),
                                // grabs stage note and shoots
                                new ParallelDeadlineGroup(new SequentialCommandGroup(new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(new WaitCommand(0.1),
                                                                new AutoIntake(intake, 1.)),
                                                swerve.followPathCommand(
                                                                "6 Piece Part 1", true))
                                                .withTimeout(2.),

                                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true,
                                                                false, 0)),
                                                new AlignToGoalOnlyOdom(swerve).withTimeout(0.5),
                                                new ShootFalconForced(shooter, 1., 0.6, intake, hood, () -> 35.)),
                                                new RevUp(shooter, 1., 0.6, () -> 35., hood)),
                                // grabs middle note and shoots
                                new ParallelDeadlineGroup(new SequentialCommandGroup(new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(new WaitCommand(0.5),
                                                                new AutoIntake(intake, 1.)),
                                                swerve.followPathCommand(
                                                                "6 Piece Part 2",
                                                                false).withTimeout(3.))
                                                .withTimeout(4.0),

                                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true,
                                                                false, 0)),
                                                new AlignToGoalOnlyOdom(swerve).withTimeout(0.5),

                                                new ShootFalcon(shooter, 1., 0.6, intake, hood, () -> 39)),
                                                new RevUp(shooter, 1., 0.6, () -> 39, hood)),
                                // grabs and shoots amp note
                                new ParallelDeadlineGroup(new SequentialCommandGroup(new ParallelDeadlineGroup(
                                                new SequentialCommandGroup(new WaitCommand(0.5),
                                                                new AutoIntake(intake, 1.)),
                                                swerve.followPathCommand(
                                                                "6 Piece Part 3",
                                                                false))
                                                .withTimeout(4.0),

                                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true,
                                                                false, 0)),
                                                new AlignToGoalOnlyOdom(swerve).withTimeout(0.5),

                                                new ShootFalcon(shooter, 1., 0.6, intake, hood, () -> 33.)),
                                                new RevUp(shooter, 1., 0.6, () -> 33., hood))

                );
        }
}
