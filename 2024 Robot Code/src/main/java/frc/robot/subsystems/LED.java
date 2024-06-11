// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {

    private static LED mInstance;

    public static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    private final CANdle mLED;
    private boolean isAuto;

    private CANdleConfiguration config;
    public boolean isAnimating = false;
    public boolean isShooting = false;
    public boolean isAligned = false;

    /** Creates a new LED. */
    public LED() {
        isAuto = false;
        mLED = new CANdle(Constants.LED.LED_PORT, Constants.CANIVORE_NAME);
        config = new CANdleConfiguration();
        config.statusLedOffWhenActive = false;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.5;
        config.vBatOutputMode = VBatOutputMode.On;

        mLED.configAllSettings(config, 100);
        // mLED.configFactoryDefault();

    }

    public void setAutoLED() {
        isAuto = true;
    }

    public void setTeleOpLED() {
        isAuto = false;
    }

    public boolean isUsingAutoLED() {
        return isAuto;
    }

    public void setAmpLEDs(Color color) {
        mLED.clearAnimation(0);
        mLED.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 255,
                Constants.LED.AMP_LED_START_INDEX,
                Constants.LED.AMP_LED_END_INDEX);
    }

    public void setLeftAmpLEDs(Color color) {
        mLED.clearAnimation(0);
        mLED.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 255,
                Constants.LED.AMP_LED_START_INDEX,
                (Constants.LED.AMP_LED_END_INDEX - Constants.LED.AMP_LED_START_INDEX + 1) / 2);
    }

    public void setRightAmpLEDs(Color color) {
        mLED.clearAnimation(0);
        mLED.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 255,
                (Constants.LED.AMP_LED_END_INDEX - Constants.LED.AMP_LED_START_INDEX + 1) / 2 + 1,
                (Constants.LED.AMP_LED_END_INDEX - Constants.LED.AMP_LED_START_INDEX + 1) / 2);
    }

    public void setCANDLELED(Color color) {
        mLED.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 255,
                Constants.LED.CANDLE_START_INDEX,
                Constants.LED.CANDLE_LED_LENGTH);
    }

    public void setCANDLELEDIndex(Color color, int idx) {
        mLED.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 255,
                Constants.LED.CANDLE_START_INDEX + idx,
                1);
    }

    /**
     * @return bus voltage
     */
    public double getBatteryVoltage() {
        return mLED.getBusVoltage();
    }

    /**
     * @return rail voltage
     */
    public double get5V() {
        return mLED.get5VRailVoltage();
    }

    /**
     * @return low side current
     */
    public double getCurrent() {
        return mLED.getCurrent();
    }

    /**
     * @return temperature in celcius
     */
    public double getTemperature() {
        return mLED.getTemperature();
    }

    /**
     * @param percent scaling brightness
     */
    public void configBrightness(double percent) {
        mLED.configBrightnessScalar(percent, 0);
    }

    /**
     * @param disableWhenLos whether to disable LED when signal is lost
     */
    public void configLos(boolean disableWhenLos) {
        mLED.configLOSBehavior(disableWhenLos, 0);
    }

    /**
     * @param type the type of LED
     */
    public void configLedType(LEDStripType type) {
        mLED.configLEDType(type, 0);
    }

    /**
     * @param offWhenActive whether LED is off when CANdle is activated
     */
    public void configStatusLedBehavior(boolean offWhenActive) {
        mLED.configStatusLedState(offWhenActive, 0);
    }

    public static double last_time = Timer.getFPGATimestamp();
    public static int cor = 0;

    public boolean goingDown = false;
    public int cycleIndex = 0;
    public int[] backLEDDisplay = { 3, 4, 1 };
    public double actualLastTime = Timer.getFPGATimestamp();

    double lastIntake = -1.0;

    public void updateAnimation(boolean intakeBeamBreak, boolean bottomIntake, boolean leftLimitSwitch,
            boolean rightLimitSwitch, boolean leftShooterRPM, boolean rightShooterRPM, boolean hoodAngle,
            boolean readyToShoot, boolean trapHasPiece) {
        if (!Constants.realRobot)
            return;

        /**
         * when we pick up a piece, flash (done i think)
         * status signal for sensors (done i think)
         * red while aligning, green while good to shoot
         * signal piece to human player
         * when we balance, we do an animation
         * when we are disabled, we do an animation
         */
        if (!intakeBeamBreak) {
            LimelightInterface.getInstance().setLimeLightLED(1);
        }
        if (intakeBeamBreak)
            this.setCANDLELEDIndex(Color.kGreen, 5);
        else
            this.setCANDLELEDIndex(Color.kRed, 5);
        if (bottomIntake) {
            this.setCANDLELEDIndex(Color.kGreen, 2);

        } else
            this.setCANDLELEDIndex(Color.kRed, 2);

        if (leftLimitSwitch)
            this.setCANDLELEDIndex(Color.kGreen, 4);
        else
            this.setCANDLELEDIndex(Color.kRed, 4);

        if (rightLimitSwitch)
            this.setCANDLELEDIndex(Color.kGreen, 3);
        else
            this.setCANDLELEDIndex(Color.kRed, 3);

        if (leftShooterRPM)
            this.setCANDLELEDIndex(Color.kGreen, 6);
        else
            this.setCANDLELEDIndex(Color.kRed, 6);
        if (rightShooterRPM)
            this.setCANDLELEDIndex(Color.kGreen, 1);
        else
            this.setCANDLELEDIndex(Color.kRed, 1);
        if (hoodAngle)
            this.setCANDLELEDIndex(Color.kGreen, 0);
        else
            this.setCANDLELEDIndex(Color.kRed, 0);

        if (trapHasPiece)
            this.setCANDLELEDIndex(Color.kGreen, 7);
        else
            this.setCANDLELEDIndex(Color.kRed, 7);

        if (!(Swerve.getInstance().isEnabled || Swerve.getInstance().isTeleop))
            return;

        if (!intakeBeamBreak)
            lastIntake = -1;

        boolean flag = false;
        if (LimelightInterface.getInstance().hasTarget("limelight-host")
                &&
                leftShooterRPM && rightShooterRPM
                && isAligned) {
            this.setAmpLEDs(Color.kGreen);
        } else if (intakeBeamBreak && (Timer.getFPGATimestamp() - lastIntake <= 2.0 || lastIntake == -1)) {
            flag = true;
            if (lastIntake == -1.0)
                lastIntake = Timer.getFPGATimestamp();
            this.getLED()
                    .animate(new StrobeAnimation((int) (Color.kNavajoWhite.red * 255.),
                            (int) (Color.kNavajoWhite.green * 255.),
                            (int) (Color.kNavajoWhite.blue * 255.), 255, 0.1, Constants.LED.LED_STRING_LENGTH,
                            Constants.LED.AMP_LED_START_INDEX), 0);
            if ((int) ((((Timer.getFPGATimestamp() - lastIntake) - (int) (Timer.getFPGATimestamp() - lastIntake))
                    * 100)) % 20 < 10) {
                LimelightInterface.getInstance().setLimeLightLED(3);

            } else {
                LimelightInterface.getInstance().setLimeLightLED(1);

            }

        } else if (bottomIntake) {
            setAmpLEDs(Color.kYellow);

        } else if (rightLimitSwitch && leftLimitSwitch && Climber.getInstance().isClimbed) {
            flag = true;

            this.getLED()
                    .animate(new RainbowAnimation(1., 1.,
                            Constants.LED.LED_STRING_LENGTH - Constants.LED.AMP_LED_START_INDEX + 1,
                            false, Constants.LED.AMP_LED_START_INDEX), 0);

        } else if (LimelightInterface.getInstance().hasTarget("limelight-host")

                && isAligned) {

            this.setAmpLEDs(Color.kPurple);

        } else if (LimelightInterface.getInstance().hasTarget("limelight-host")) {

            this.setAmpLEDs(Color.kRed);

        } else
            this.setAmpLEDs(Color.kBlack);

        // if (rightLimitSwitch && leftLimitSwitch && Climber.isClimbed) {
        // flag = true;

        // this.getLED()
        // .animate(new RainbowAnimation(1., 0.5,
        // Constants.LED.LED_STRING_LENGTH - Constants.LED.AMP_LED_START_INDEX + 1,
        // false, Constants.LED.AMP_LED_START_INDEX), 0);
        // }

        if (LimelightInterface.getInstance().hasTarget("limelight-tag")
                &&
                leftShooterRPM && rightShooterRPM
                && isAligned) {
            this.setAmpLEDs(Color.kGreen);
        }
        if (!flag)
            clearAnimation();

    }

    private void logToDashboard() {

    }

    private void logToDashboardInDetail() {
        SmartDashboard.putNumber("LED/Current", getCurrent());
        SmartDashboard.putNumber("LED/Temperature", getTemperature());
        SmartDashboard.putNumber("LED/Battery Voltage", getBatteryVoltage());
        SmartDashboard.putNumber("LED/5V Rail Voltage", get5V());

    }

    /**
     * @return LED
     */
    public CANdle getLED() {
        return mLED;
    }

    @Override
    public void periodic() {
        
        if (!Constants.realRobot)
            return;

        updateAnimation(Intake.getInstance().isInConveyor(), Intake.getInstance().isInIntake(),
                Climber.getInstance().getLeftLimit(), Climber.getInstance().getRightLimit(),
                Shooter.getInstance().atSpeedLeft(),
                Shooter.getInstance().atSpeedRight(),
                ShooterHood.getInstance().atPosition(),
                Shooter.getInstance().atSpeed() && ShooterHood.getInstance().atPosition(), false); // TODO FIX

    }

    public void clear() {
        getLED().clearAnimation(0);
        setAmpLEDs(Color.kBlack);
    }

    public void clearAnimation() {
        getLED().clearAnimation(0);

    }

}