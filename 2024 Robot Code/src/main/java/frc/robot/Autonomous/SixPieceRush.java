// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.commands.Shooter.RevUp;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixPieceRush extends SequentialCommandGroup {
        /** Creates a new SixPiece. */
        public SixPieceRush(Swerve swerve, ShooterHood hood, Shooter shooter, Intake intake) {
                // Add your commands in the addCommands() call, e.g.
                addCommands(
                                new ThreePieceRushTop(swerve, hood, shooter, intake),
                                new ParallelDeadlineGroup(new SequentialCommandGroup(new WaitCommand(0.75),
                                                new AutoIntake(intake, 1.)),
                                                swerve.followPathCommand(
                                                                "Rush 6",
                                                                false)),
                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false, 0)),

                                new ParallelDeadlineGroup(
                                                swerve.followPathCommand(
                                                                "Rush 7",
                                                                false),
                                                new RevUp(shooter, 1., .6, () -> 39.,
                                                                hood))
                                                .withTimeout(2.75),
                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false, 0))

                                
                );
        }
}
