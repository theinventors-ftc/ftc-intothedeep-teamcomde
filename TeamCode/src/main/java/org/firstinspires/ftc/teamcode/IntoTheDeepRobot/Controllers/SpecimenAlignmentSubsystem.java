package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.controllers.IIRSubsystem;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;

import java.util.function.DoubleSupplier;

@Config
public class SpecimenAlignmentSubsystem extends SubsystemBase {
    private Limelight3A limelight;
    private LLResult result;
    private double target = 320;
    private boolean enabled = true;
    public static double kP = 0.005, kI = 0, kD = 3, A = 0.1, ALPHA = 0.05;
    PIDFControllerEx controller;
    private Telemetry telemetry;

    private IIRSubsystem filter;

    public static double clip = 0.5;
    public SpecimenAlignmentSubsystem(Limelight3A limelight3A,
                                      Telemetry telemetry) {
        controller = new PIDFControllerEx(kP, kI, kD, 0, A, 0.5, 1, 0.7);
        this.limelight = limelight3A;
        limelight.start();

        this.telemetry = telemetry;
        telemetry.setMsTransmissionInterval(11);

        filter = new IIRSubsystem(ALPHA, this::getSpecX);
    }

    @Override
    public void periodic() {
        filter.set(ALPHA);
        setDistTarget(target);
        controller.setPIDF(kP, kI, kD, 0);
        controller.setAlpha(A);
        telemetry.addData("Specimen Center Filtered: ", filter.get());
        telemetry.addData("Target Specimen Center", target);
        telemetry.addData("Specimen Center: ", getSpecX());
        telemetry.addData("Enabled: ", enabled);
    }

    public double getSpecX() {
        if (limelight.getLatestResult() == null) return target;

        return limelight.getLatestResult().getPythonOutput()[0];
    }

    public double calculatePower() {
//        double clip_map =
        return Range.clip(controller.calculate(), -clip, clip);
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