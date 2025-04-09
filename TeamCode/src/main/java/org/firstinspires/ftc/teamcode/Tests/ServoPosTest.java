package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.controllers.IIRSubsystem;

@Config
@TeleOp(name = "Servo Pos Test", group = "Tests")
public class ServoPosTest extends LinearOpMode {
    public static double servoValue = 0.0;
    private ServoImplEx servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(ServoImplEx.class, "intake_raise_right");

        waitForStart();

        while (opModeIsActive()) {
            servo.setPosition(servoValue);
        }
    }
}
