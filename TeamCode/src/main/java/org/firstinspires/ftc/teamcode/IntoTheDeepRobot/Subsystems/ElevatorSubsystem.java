package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

@Config
public class ElevatorSubsystem extends SubsystemBase {
    private MotorExEx elevatorMotor, elevatorMotorFollow;
    private final double MAX_ELEVATOR_POWER = 1.0;
    private final int MAX_ELEVATOR_HEIGHT = 0;
    private DoubleSupplier power;

    private ElevatorFeedforward ff = new ElevatorFeedforward(
            0.1,
            0.16,
            1.0,
            0.0
    );
    private PIDFControllerEx pid = new PIDFControllerEx(
            0.007,
            0.008,
            0.0001,
            0.0,
            0,
            0.0,
            80,
            1
    );

    public enum Level {
        INTAKE,
        SPECIMEN_DISLOCATE,
        PARK,
        PARK2,
        LOW_BASKET,
        HIGH_BASKET,
        LOW_CHAMBER,
        HIGH_CHAMBER,
        HANGING_AIM,
        HANGING,
        MANUAL
    }

    private Level level;

    HashMap<Level, Integer> levelMap = new HashMap<Level, Integer>() {{
        put(Level.INTAKE, 0);
        put(Level.SPECIMEN_DISLOCATE, 150);
        put(Level.PARK, 250);
        put(Level.PARK2, 500);
        put(Level.LOW_BASKET, 740);
        put(Level.HIGH_BASKET, 2245);
        put(Level.LOW_CHAMBER, 0);
        put(Level.HIGH_CHAMBER, 565);
        put(Level.HANGING_AIM, 1000);
        put(Level.HANGING, 0);
    }};

    public final double ratio = 20.0/24.0;

    public static int target_height = 0;
    private int springs_off = 34;

    // Zeroing
    public boolean isStalled = false;
    public double ampThreshold = 3.4;
    private final Timing.Timer timer;
    private boolean found_zero = false;

    private Telemetry telemetry;

    public ElevatorSubsystem(RobotMap robotMap, DoubleSupplier power, Telemetry telemetry) {
        elevatorMotor = robotMap.getSliderMotor();
        elevatorMotorFollow = robotMap.getSliderFollow();
        elevatorMotor.setRunMode(MotorExEx.RunMode.RawPower);
        elevatorMotor.setInverted(true);
        elevatorMotorFollow.setRunMode(MotorExEx.RunMode.RawPower);
        elevatorMotor.resetEncoder();
        elevatorMotorFollow.resetEncoder();

        this.power = power;

        timer = new Timing.Timer(600, TimeUnit.MILLISECONDS);

        this.telemetry = telemetry;

        setLevel(Level.INTAKE);
    }

    private void set(double power) {
        power *= MAX_ELEVATOR_POWER;
        double ffPower = ff.calculate(power);
        elevatorMotor.set(ffPower);
        elevatorMotorFollow.set(ffPower);
    }

    public void setLevel(Level level) {
        this.level = level;
        target_height = (int)(levelMap.get(level)*ratio);
    }

    @Override
    public void periodic() {
        if (!found_zero) {
            searchZero();
            return;
        }

        pid.setSetPoint(target_height);

        telemetry.addData("Cur Height: ", getHeight());

        if(Math.abs(power.getAsDouble()) > 0.05){ // Manual
            set(power.getAsDouble());
            target_height = getHeight();
            level = Level.MANUAL;
        } else { // Auto
            set(pid.calculate(getHeight())); // TODO: TUNE FOR NEW MOTORS/RATIO
        }
    }

    public int getHeight() {
        return elevatorMotor.getCurrentPosition()-springs_off;
    }

    public Level getLevel() {
        return level;
    }

    // ----------------------------------------- Zeroing ---------------------------------------- //

    public void searchZero() {
        if (!isSliderBottom()) {
            set(-0.6);

            if(getCurrent() > ampThreshold && !timer.isTimerOn()) {
                timer.start();
            }

            if(timer.done() && getCurrent() > ampThreshold){
                isStalled = true;
            }
        } else {
            set(0);
            elevatorMotor.resetEncoder();
            elevatorMotorFollow.resetEncoder();
            found_zero = true;
        }
    }

    public double getCurrent() {
        return ((DcMotorEx)elevatorMotor.getRawMotor()).getCurrent(CurrentUnit.AMPS);
    }

    public boolean isSliderBottom() {
        return isStalled;
    }
}
