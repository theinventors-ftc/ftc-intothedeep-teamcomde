package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

@Config
public class HeadingControllerSubsystem extends SubsystemBase {
    private DoubleSupplier gyroValue;
    private IntSupplier closestOrientationTarget;

    private double target = 0;
    public static double TARGET = 0;

    private boolean enabled = true;
    private boolean findClosestTarget;

    public static double kP = 0.025, kI = 0, kD = 0.003, A = 0.2;

    PIDFControllerEx controller;

    private Telemetry telemetry;

    public HeadingControllerSubsystem(DoubleSupplier gyroValue,
                                      IntSupplier closestOrientationTarget,
                                      Telemetry telemetry) {
        controller = new PIDFControllerEx(kP, kI, kD, 0, A, 0.35, 0, 0);
        this.gyroValue = gyroValue;
        this.closestOrientationTarget = closestOrientationTarget;

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        controller.setPIDF(kP, kI, kD, 0);
        controller.setAlpha(A);
        telemetry.addData("Gyro Value", gyroValue.getAsDouble()%360);
        telemetry.addData("Target", TARGET);
//        telemetry.update();
    }

    public double calculateTurn() {
        if (findClosestTarget) {
            target = closestOrientationTarget.getAsInt();
            findClosestTarget = false;
        }

        return controller.calculate(gyroValue.getAsDouble());
    }


    public boolean isEnabled() {
        return enabled;
    }

    public void setGyroTarget(double targetOrient) {
        double gyroValueDouble = gyroValue.getAsDouble();
        int minDistIdx, maxIdx;

        maxIdx = (int) Math.ceil(gyroValueDouble / 360);
        if (Math.abs((maxIdx - 1) * 360 + targetOrient - gyroValueDouble) > Math.abs((maxIdx) * 360 + targetOrient - gyroValueDouble))
            minDistIdx = maxIdx;
        else
            minDistIdx = maxIdx - 1;

        target = minDistIdx * 360 + targetOrient;
        controller.setSetPoint(target);
    }

    public double getTarget() {
        return target;
    }

    public void toggleState() {
        enabled = !enabled;
        findClosestTarget = enabled || findClosestTarget;
    }

    public void enable() {
        enabled = true;
        findClosestTarget = enabled || findClosestTarget;
    }

    public void disable() {
        enabled = false;
        findClosestTarget = enabled || findClosestTarget;
    }
}