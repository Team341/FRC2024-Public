// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Shooter.ShootFalcon;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootRealign extends SequentialCommandGroup {
        /** Creates a new SixPiece. */

        // Assuming this is a method in your drive subsystem

        public ShootRealign(Swerve swerve, ShooterHood hood, Shooter shooter, Intake intake, boolean isAmpSide) {
                // Add your commands in the addCommands() call, e.g.
                Command zipZap = isAmpSide ? (new WaitCommand(1.)) : swerve.followPathCommand("Zip Out", true);
                addCommands(
                                new ShootFalcon(shooter, 1., 0.6, intake, hood,
                                                () -> Constants.Shooter.SPEAKER_CLOSE_ANGLE).withTimeout(4.),
                                zipZap.withTimeout(4.),
                                new InstantCommand(() -> swerve.drive(new Translation2d(), 0, true, false, 0))

                                // new InstantCommand(() -> swerve.setYaw(bangle), swerve)

                );
        }
}
