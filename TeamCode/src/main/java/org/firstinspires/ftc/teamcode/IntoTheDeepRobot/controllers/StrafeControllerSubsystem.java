package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.controllers.IIRSubsystem;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;

import java.util.function.DoubleSupplier;

@Config
public class StrafeControllerSubsystem extends SubsystemBase {
    private DoubleSupplier distValue;
    private double target = 0;
    private boolean enabled = false;
    public static double kP = 0.16, kI = 0, kD = 0.006, A = 0.6, ALPHA = 0.05;
    PIDFControllerEx controller;
    private Telemetry telemetry;

    private IIRSubsystem filter;

    public StrafeControllerSubsystem(DoubleSupplier distValue,
                                     Telemetry telemetry) {
        controller = new PIDFControllerEx(kP, kI, kD, 0, A, 0.5, 1, 0.7);
        this.distValue = distValue;
        filter = new IIRSubsystem(ALPHA, distValue);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        filter.set(ALPHA);
//        setDistTarget(target);
        controller.setPIDF(kP, kI, kD, 0);
        controller.setAlpha(A);
        telemetry.addData("Dist Value", filter.get());
        telemetry.addData("Target Distance", target);
        telemetry.update();
    }

    public double calculatePower() {
        return controller.calculate(filter.get());
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setDistTarget(double targetDist) {
        target = targetDist;
        controller.setSetPoint(target);
    }

    public double getTarget() {
        return target;
    }

    public void toggleState() {
        enabled = !enabled;
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }
}