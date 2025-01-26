package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.util.Urm09;
import org.inventors.ftc.robotbase.controllers.IIRSubsystem;

//@Disabled
@Config
@TeleOp(name = "Urm09 Test", group = "Tests")
public class Urm09Test extends LinearOpMode {
    AnalogInput distanceSensor;
    IIRSubsystem iirSubsystem;
    public static double ALPHA = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(AnalogInput.class, "left_dist");
        iirSubsystem = new IIRSubsystem(ALPHA, () -> (distanceSensor.getVoltage()/3.3) * 500);

        Telemetry dash_telemetry = FtcDashboard.getInstance().getTelemetry();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            iirSubsystem.set(ALPHA);
            iirSubsystem.periodic();
            dash_telemetry.addData("Distance", iirSubsystem.get_raw());
            dash_telemetry.addData("Filt Distance", iirSubsystem.get());
            dash_telemetry.update();
        }
    }
}
