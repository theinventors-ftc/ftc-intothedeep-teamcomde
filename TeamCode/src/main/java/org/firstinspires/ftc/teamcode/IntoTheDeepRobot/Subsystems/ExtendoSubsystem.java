package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class ExtendoSubsystem extends SubsystemBase {
    private MotorExEx extendoMotor;
    private double MAX_EXTENDO_POWER = 1.0;
    private int MAX_EXTENSION = 1800; // Allowed: 1200, Max: 1800
    private DoubleSupplier power;

    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
            0.0,
            1.0,
            0.0
    );
    private PIDFControllerEx pid = new PIDFControllerEx(
            0.007,
            0.0, // 0.06
            0.06,
            0.0,
            0.0,
            0.0,
            80,
            0.6
    );

    public static int targetPosition = 0;
    private int springs_off = 25;

    private Telemetry telemetry;

    // Zeroing
    public boolean isStalled = false;
    public double ampThreshold = 4;
    private final Timing.Timer timer;
    private boolean found_zero = false;

    //
    private boolean block_manual = false;


    public ExtendoSubsystem(
            RobotMap robotMap,
            DoubleSupplier power,
            Telemetry telemetry,
            boolean attempt_Zero
    ) {
        extendoMotor = robotMap.getExtendoMotor();
        extendoMotor.setRunMode(MotorExEx.RunMode.RawPower);
        extendoMotor.resetEncoder();
        this.power = power;

        this.telemetry = telemetry;

        timer = new Timing.Timer(600, TimeUnit.MILLISECONDS);

        found_zero = !attempt_Zero;

        springs_off = attempt_Zero ? springs_off : 0;
        targetPosition = -springs_off;
    }

    private void set(double power) {
        extendoMotor.set(Range.clip(ff.calculate(power), -MAX_EXTENDO_POWER,  MAX_EXTENDO_POWER));
    }

    public void set_MAX_POWER(double power) {
        MAX_EXTENDO_POWER = power;
    }

    @Override
    public void periodic() {
        if (!found_zero) {
            searchZero();
            return;
        }

        pid.setSetPoint(Range.clip(targetPosition, 90, MAX_EXTENSION));

        telemetry.addData("Extendo Position", getExtension());

        if(Math.abs(power.getAsDouble()) > 0.05 && !block_manual){ // Manual
            set(Range.clip(
                    power.getAsDouble(),
                    getExtension() > 90 ? -1 : 0.0,
                    getExtension() < MAX_EXTENSION ? 1 : 0.0));
            targetPosition = Range.clip(getExtension(), 90, MAX_EXTENSION);
        } else { // Auto
            set(pid.calculate(getExtension()));
        }
    }

    public void setTargetPosition(int position) {
        targetPosition = Range.clip(position, 90, MAX_EXTENSION);
    }

    public void returnToZero() {
        setTargetPosition(0);
    }

    public boolean atTarget() {
        return Math.abs(getExtension() - targetPosition) < 30;
    }

    public int getExtension() {
        return extendoMotor.getCurrentPosition()-springs_off;
    }

    // Zeroing
    public void searchZero() {
        if (!isExtendoBack()) {
            set(-0.6);

            if(getCurrent() > 4.5 && !timer.isTimerOn()) {
                timer.start();
            }

            if(timer.done() && getCurrent() > ampThreshold){
                isStalled = true;
            }
        } else {
            set(0);
            extendoMotor.resetEncoder();
            found_zero = true;
        }
    }

    public double getCurrent() {
        return ((DcMotorEx)extendoMotor.getRawMotor()).getCurrent(CurrentUnit.AMPS);
    }

    public boolean isExtendoBack() {
        return isStalled;
    }

    public void blockManual(boolean block) {
        block_manual = block;
    }
}
