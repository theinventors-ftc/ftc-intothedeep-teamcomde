package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;

import java.util.function.DoubleSupplier;

@Config
public class ForwardControllerSubsystem extends SubsystemBase {
    private DoubleSupplier distValue;
    private double target = 0;
    private boolean enabled = false;
    public static double kP = 0.06, kI = 0, kD = 0.003, A = 0.2;
    PIDFControllerEx controller;
    private Telemetry telemetry;

    public ForwardControllerSubsystem(DoubleSupplier distValue,
                                      Telemetry telemetry) {
        controller = new PIDFControllerEx(kP, kI, kD, 0, A, 0.0, 0, 0);
        this.distValue = distValue;

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        setGyroTarget(target);
        controller.setPIDF(kP, kI, kD, 0);
        controller.setAlpha(A);
        telemetry.addData("Dist Value", distValue.getAsDouble());
        telemetry.addData("Target Distance", target);
        telemetry.update();
    }

    public double calculatePower() {
        return controller.calculate(distValue.getAsDouble());
    }


    public boolean isEnabled() {
        return enabled;
    }

    public void setGyroTarget(double targetDist) {
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