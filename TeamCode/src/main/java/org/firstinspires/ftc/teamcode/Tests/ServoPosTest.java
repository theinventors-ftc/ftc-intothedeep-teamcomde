package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Disabled
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
