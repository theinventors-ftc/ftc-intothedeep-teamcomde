package org.firstinspires.ftc.teamcode.Tests;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.util.MotorPair;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

@TeleOp(name = "Motor Sync Test", group = "Tests")
public class MotorSyncTest extends LinearOpMode {
    MotorExEx leadMotor, followMotor;
    MotorPair motorPair;

    @Override
    public void runOpMode() throws InterruptedException {
        leadMotor = new MotorExEx(hardwareMap, "lead", MotorExEx.GoBILDA.RPM_435);
        followMotor = new MotorExEx(hardwareMap, "follow", MotorExEx.GoBILDA.RPM_435);

        leadMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        followMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motorPair = new MotorPair(leadMotor, followMotor, 96.0/24.0);

        motorPair.lead_setInverted(false);
        motorPair.follow_setInverted(true);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;

            motorPair.set(power);
        }
    }
}
