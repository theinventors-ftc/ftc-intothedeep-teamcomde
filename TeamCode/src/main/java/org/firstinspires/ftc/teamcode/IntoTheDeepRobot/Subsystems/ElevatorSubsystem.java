package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

@Config
public class ElevatorSubsystem extends SubsystemBase {
    private MotorExEx elevatorMotor, elevatorMotorFollow, couplingMotor;
    private final double MAX_ELEVATOR_POWER = 1.0;
    private final int MAX_ELEVATOR_HEIGHT = 2350;
    private DoubleSupplier power;

    private ElevatorFeedforward ff = new ElevatorFeedforward(
            0.08,
            0.16,
            1.0,
            0.0
    );
    private PIDFControllerEx pid = new PIDFControllerEx(
            0.008,
            0.008, // 0.008
            0.0002,
            0.0,
            0,
            0.0,
            80,
            1
    );

    public enum Level {
        INTAKE,
        SPECIMEN_DISLOCATE,
        PARK0,
        PARK,
        PARK2,
        LOW_BASKET,
        HIGH_BASKET,
        LOW_CHAMBER,
        HIGH_CHAMBER,
        HANGING_AIM,
        HANGING,
        HANGING_RELEASE,
        HIGH_CHAMBER_RELEASE,
        MANUAL
    }

    private Level level;

    HashMap<Level, Integer> levelMap = new HashMap<Level, Integer>() {{
        put(Level.INTAKE, 5);
        put(Level.SPECIMEN_DISLOCATE, 137);
        put(Level.PARK0, 80);
        put(Level.PARK, 230);
        put(Level.PARK2, 450);
        put(Level.LOW_BASKET, 680);
        put(Level.HIGH_BASKET, 2058);
        put(Level.LOW_CHAMBER, 0);
        put(Level.HIGH_CHAMBER, 518);
        put(Level.HIGH_CHAMBER_RELEASE, 0);
        put(Level.HANGING_AIM, 917);
        put(Level.HANGING, -165);
        put(Level.HANGING_RELEASE, -10); // TODO Go intake in auto hang
    }};

    public static int target_height = 0;
    private int springs_off = 30;

    // Zeroing
    public boolean isStalled = false;
    public double ampThreshold = 4;
    private final Timing.Timer timer;
    private boolean found_zero = false, attempt_Zero = false;

    // Coupler
    private boolean coupled = false;

    private Telemetry telemetry;

    private GamepadExEx toolOp;

    public ElevatorSubsystem(RobotMap robotMap, DoubleSupplier power, MotorExEx couplingMotor,
                             Telemetry telemetry, boolean attempt_Zero) {
        elevatorMotor = robotMap.getSliderMotor();
        elevatorMotorFollow = robotMap.getSliderFollow();
        elevatorMotor.setRunMode(MotorExEx.RunMode.RawPower);
        elevatorMotor.setInverted(true);
        elevatorMotorFollow.setRunMode(MotorExEx.RunMode.RawPower);
        elevatorMotor.resetEncoder();
        elevatorMotorFollow.resetEncoder();
        this.couplingMotor = couplingMotor;

        this.power = power;
        this.toolOp = robotMap.getToolOp();

        timer = new Timing.Timer(600, TimeUnit.MILLISECONDS);

        this.telemetry = telemetry;

        setLevel(Level.INTAKE);

        springs_off = attempt_Zero ? springs_off : 0;
        target_height = -springs_off;

        found_zero = !attempt_Zero;
    }

    private void set(double power) {
        power *= MAX_ELEVATOR_POWER;
        double ffPower = ff.calculate(power);
        if(coupled) { // This syncs the elevator and couples
            elevatorMotor.set(ffPower*48.0/60.0);
            elevatorMotorFollow.set(ffPower*48.0/60.0);
            couplingMotor.set(ffPower);
        } else {
            elevatorMotor.set(ffPower);
            elevatorMotorFollow.set(ffPower);
        }
    }

    public void setLevel(Level level) {
        this.level = level;
        target_height = (int)(levelMap.get(level));
    }

    @Override
    public void periodic() {
//        if (!found_zero) {
//            searchZero();
//            return;
//        }

        pid.setSetPoint(Range.clip(target_height, -250, MAX_ELEVATOR_HEIGHT));
//        pid.setSetPoint(target_height);

        telemetry.addData("Cur Height: ", getHeight());
        telemetry.addData("Target Height: ", target_height);
        telemetry.addData("Power: ", power.getAsDouble());



        if(Math.abs(power.getAsDouble()) > 0.05){ // Manual
            set(power.getAsDouble());
            target_height = getHeight();
            level = Level.MANUAL;
        } else { // Auto
            set(pid.calculate(getHeight())); // TODO: TUNE FOR NEW MOTORS/RATIO
        }
    }

    public int getHeight() {
        return (int)((elevatorMotor.getCurrentPosition()-springs_off)*(22.0/20.0));
    }

    public Level getLevel() {
        return level;
    }

    public boolean atTarget() {
        return Math.abs(getHeight() - target_height) < 20;
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

    public void setCoupled(boolean coupled) {
        this.coupled = coupled;
    }

    public void disable() {
//        elevatorMotor.disable();
//        elevatorMotorFollow.disable();
//        couplingMotor.disable();
        setLevel(Level.HANGING_RELEASE);
    }
}
