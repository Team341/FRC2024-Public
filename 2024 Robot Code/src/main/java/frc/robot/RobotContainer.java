// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileNotFoundException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.FivePieceCenter;
import frc.robot.Autonomous.FourPiece;
import frc.robot.Autonomous.RushConditionalThree;
import frc.robot.Autonomous.ShootRealign;
import frc.robot.Autonomous.SixPiece;
import frc.robot.Autonomous.SixPieceSlide;
import frc.robot.Autonomous.SourceRushCenter;
import frc.robot.Autonomous.ThreePieceRushTop;
import frc.robot.commands.FullReset;
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Climber.ClimbUp;
import frc.robot.commands.Drive.AlignToGoalOnlyOdom;
import frc.robot.commands.Drive.ResetGyro;
import frc.robot.commands.Drive.TeleopSwerve;
import frc.robot.commands.Intake.TeleIntake;
import frc.robot.commands.Intake.UnTeleIntake;
import frc.robot.commands.Shooter.RevUpAmp;
import frc.robot.commands.Shooter.RevUpTeleop;
import frc.robot.commands.Shooter.ShootFalconAmp;
import frc.robot.commands.Shooter.ShootFalconInstant;
import frc.robot.commands.Shooter.ShootFalconSetSpeed;
import frc.robot.commands.Shooter.ShootFalconSetSpeedInstant;
import frc.robot.commands.Shooter.ShootFalconTeleop;
import frc.robot.subsystems.AmpBar;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
// import frc.robot.subsystems.LED;
import frc.robot.subsystems.LimelightInterface;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        public final XboxController mDriverController = new XboxController(
                        Constants.ControllerInputs.DRIVER_CONTROLLER_PORT);
        public final XboxController mOperatorController = new XboxController(
                        Constants.ControllerInputs.OPERATOR_CONTROLLER_PORT);
       
        /* Controllers */
        private final Joystick driver = new Joystick(0);
        /* Drive Controls */
        private final int translationAxis = XboxController.Axis.kLeftY.value;
        private final int strafeAxis = XboxController.Axis.kLeftX.value;
        private final int rotationAxis = XboxController.Axis.kRightX.value;
        private final int otherAxis = XboxController.Axis.kRightY.value;

        /* Driver Buttons */

        /* Subsystems */
        private final Swerve s_Swerve = Swerve.getInstance();
        private final Shooter mShooter = Shooter.getInstance();
        private AmpBar mAmp = AmpBar.getInstance();
 
        private Intake mIntake = Intake.getInstance();
        private ShooterHood mHood = ShooterHood.getInstance();
        private LimelightInterface mLimelightInterface = LimelightInterface.getInstance();
        private LED mLED = LED.getInstance();
        public SendableChooser<Command> mAutonomousChooser = new SendableChooser<>();
        public boolean builtChooser = false;
        public boolean isFirstTeleop = false;

        private Climber mClimber = Climber.getInstance();

        public void buildAutoChooser() {
                builtChooser = true;

                mAutonomousChooser.setDefaultOption("Five.Five Amp-Side (auto-aligns, no chloreo, yada yada)",
                                new SixPiece(s_Swerve, mHood, mShooter, mIntake));

                mAutonomousChooser.addOption("Three-point-Five Piece Rush Amp-Side w/Slide (1 and 2)",
                                new RushConditionalThree(s_Swerve, mHood, mShooter, mIntake));
                mAutonomousChooser.addOption("Three Piece Rush Amp-Side (2 & 3)",
                                new ThreePieceRushTop(s_Swerve, mHood, mShooter, mIntake));
                 mAutonomousChooser.addOption("Three Piece Rush Source-Side ",
                                new SourceRushCenter(s_Swerve, mHood, mShooter, mIntake));

          
                mAutonomousChooser.addOption("Six Piece CHOREO (das ist gut)",
                                new SixPieceSlide(s_Swerve, mHood, mShooter, mIntake));

              
                mAutonomousChooser.addOption("Four Piece",
                                new FourPiece(s_Swerve, mHood, mShooter, mIntake));
                mAutonomousChooser.addOption("Five Piece Center",
                                new FivePieceCenter(s_Swerve, mHood, mShooter, mIntake));

                mAutonomousChooser.addOption("Only Shoot",

                                new ShootRealign(s_Swerve, mHood, mShooter, mIntake, true));

                mAutonomousChooser.addOption("Only Shoot and Move Out",

                                new ShootRealign(s_Swerve, mHood, mShooter, mIntake, false));
             
        }

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.l0
         */
        public RobotContainer() throws FileNotFoundException {

               

                Shuffleboard.getTab("Autonomous").add(mAutonomousChooser);

                

                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                () -> -mDriverController.getRawAxis(translationAxis),
                                                () -> -mDriverController.getRawAxis(strafeAxis),
                                                () -> -mDriverController.getRawAxis(rotationAxis),
                                                () -> false,
                                                () -> mDriverController
                                                                .getLeftBumper(),
                                                () -> mDriverController
                                                                .getRightBumper(),
                                                () -> mDriverController
                                                                .getRightTriggerAxis() > Constants.TRIGGER_THRESHOLD,
                                                () -> mDriverController.getBButton(),

                                                () -> mDriverController.getXButton(),
                                                () -> false, () -> false));

               

                mClimber.setDefaultCommand(new RunCommand(() -> {
                        mClimber.setSpeedLeft(MathUtil.applyDeadband(-mOperatorController.getRawAxis(translationAxis),
                                        Constants.STICK_DEADBAND));
                        mClimber.setSpeedRight(MathUtil.applyDeadband(-mOperatorController.getRawAxis(otherAxis),
                                        Constants.STICK_DEADBAND));
                }, mClimber));
                mHood.setDefaultCommand(new RunCommand(() -> {

                        mHood.setPositionAbsolute(mHood.getAngleFromFutureRange());
                }, mHood));
                configureBindings();

        }

        private void configureBindings() {
                

               
                Trigger mDriverStartButton = new Trigger(() -> mDriverController.getStartButton());
                mDriverStartButton.whileTrue(new ResetGyro(s_Swerve));
              

                Trigger mDriverYButton = new Trigger(() -> mDriverController.getYButton());
                mDriverYButton.whileTrue(new ShootFalconSetSpeed(mShooter, () -> 50., () -> 76., mIntake, mHood,
                                () -> Constants.Shooter.SPEAKER_CLOSE_ANGLE));
               
                Trigger mDriverAButton = new Trigger(() -> mDriverController.getAButton());
                mDriverAButton.whileTrue(new ShootFalconInstant(mShooter, 0.9, 0.9, mIntake, mHood, () -> 5.));
                
                //for tuning shots
                if (Constants.FMSDETACHED) {

                        SmartDashboard.putNumber("RC Left Shooter Target Speed", .0);
                        SmartDashboard.putNumber("RC Right Shooter Target Speed", .0);
                        SmartDashboard.putNumber("RC Hood Target Angle", 60.);
                }

                Trigger mDriverLT = new Trigger(

                                () -> (mDriverController.getLeftTriggerAxis() > Constants.TRIGGER_THRESHOLD));

                mDriverLT.whileTrue(new ShootFalconSetSpeedInstant(mShooter, 42. * 0.95, 69* 0.95, mIntake, 
                                mHood, () -> 45.)); 
               /*OPERATOR RT = REV UP */
                Trigger mOperatorRT = new Trigger(
                                () -> mOperatorController.getRightTriggerAxis() > Constants.TRIGGER_THRESHOLD);
                mOperatorRT.whileTrue(
                                new ShootFalconTeleop(mShooter, 1., 0.6, mIntake, mHood,
                                                () -> mHood.getAngleFromFutureRange()));

                /* OPERATOR RB = SHOOT AMP */
                Trigger mOperatorRB = new Trigger(
                                () -> mOperatorController.getRightBumper());
                mOperatorRB.whileTrue(new ShootFalconAmp(mShooter, 0.27, 0.27, mIntake, mHood,
                                () -> Constants.Shooter.AMP_SCORE_ANGLE));

                /* OPERATOR LT - REV UP */
                Trigger mOperatorLT = new Trigger(
                                () -> mOperatorController.getLeftTriggerAxis() > Constants.TRIGGER_THRESHOLD);
                mOperatorLT.whileTrue(new RevUpTeleop(mShooter, 1, () -> 0.6));
                /*OPERATOR Y = TOGGLE INTAKE */
                Trigger mOperatorYButton = new Trigger(() -> mOperatorController.getYButton());
                mOperatorYButton.whileTrue(new InstantCommand(() -> mIntake.toggle(), mIntake));

                Trigger mOperatorBackButton = new Trigger(() -> mOperatorController.getBackButton());
                mOperatorBackButton.whileTrue(new ClimbUp(mClimber));
                /* OPERATOR B - Stage Shot */
                Trigger mOperatorBButton = new Trigger(() -> mOperatorController.getBButton());
                mOperatorBButton.whileTrue(new ShootFalconSetSpeed(mShooter, () -> 54., () -> 90., mIntake, mHood,
                                () -> 35.));
                /* OPERATOR X - OUTTAKE */
                Trigger mOperatorXButton = new Trigger(() -> mOperatorController.getXButton());
                mOperatorXButton.whileTrue(new UnTeleIntake(mIntake, -1.));

                /*OPERATOR A: INTAKE */
                Trigger mOperatorAButton = new Trigger(() -> {
                        if (isFirstTeleop) {
                                isFirstTeleop = false;
                                return false;
                        }
                        else return mOperatorController.getAButton();
                });
                mOperatorAButton.whileTrue(new TeleIntake(mIntake, 1.));

                Trigger mOperatorStartButton = new Trigger(() -> mOperatorController.getStartButton());
                mOperatorStartButton.onTrue(new ClimbDown(mClimber));

                /* OPERATOR L - AMP BAR + REV UP */
                Trigger mOperatorLBumper = new Trigger(() -> mOperatorController.getLeftBumper());
            
                mOperatorLBumper.whileTrue(new ParallelCommandGroup(new InstantCommand(() -> mAmp.deploy(), mAmp),
                                new RevUpAmp(mShooter, 0.27, 0.27,
                                                () -> Constants.Shooter.AMP_SCORE_ANGLE, mHood)))
                                .onFalse(new InstantCommand(() -> mAmp.retract(), mAmp));

                

                Trigger mOperatorLeftJoystickPress = new Trigger(() -> mOperatorController.getLeftStickButton());

                mOperatorLeftJoystickPress.whileTrue(new FullReset(mHood, mClimber, mIntake, mAmp));
                
                SmartDashboard.putNumber("RC Left Shooter Target Speed", .0);
                SmartDashboard.putNumber("RC Right Shooter Target Speed", .0);

                Trigger mOperatorRightJoystickPress = new Trigger(() -> mOperatorController.getRightStickButton());
                
                mOperatorRightJoystickPress.whileTrue(new AlignToGoalOnlyOdom(s_Swerve));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */

        public Command getAutonomousCommand() {
                return mAutonomousChooser.getSelected();
        }
}
