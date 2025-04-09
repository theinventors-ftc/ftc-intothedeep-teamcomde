package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.util.PIDFControllerEx;

import java.util.concurrent.TimeUnit;

@Disabled
@Config
@TeleOp(name = "Axon PID Test", group = "Tests")
public class AxonPIDTest extends LinearOpMode {
    private CRServoImplEx servo;
    private AnalogInput servoPos;
    private Telemetry dashTele;
    private final PIDFControllerEx controller = new PIDFControllerEx(
            0.008,
            0.008, // 0.008
            0.0002,
            0.0,
            0,
            0.0,
            80,
            1
    );

    private Timing.Timer timer;

    public static double kP = 0.5, kI = 0.0, kD = 0.0, A = 0.0, target = 0.0, mult = 1, interv = 2000;

    public double analogToDegrees(AnalogInput analogInput) {
        return analogInput.getVoltage() / analogInput.getMaxVoltage() * 360 - 180;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dashTele = FtcDashboard.getInstance().getTelemetry();
        servo = hardwareMap.get(CRServoImplEx.class, "servo");
        servoPos = hardwareMap.get(AnalogInput.class, "servo_pos");
        controller.setSetPoint(0);
        timer = new Timing.Timer(300000, TimeUnit.MILLISECONDS);

        waitForStart();
        timer.start();

        while (opModeIsActive()) {
            controller.setPIDF(kP, kI, kD, 0);
            controller.setAlpha(A);
            controller.setSetPoint(target*mult);

            if (timer.elapsedTime() >= interv) {
                mult *= -1;
                timer.start();
            }

            servo.setPower(-controller.calculate(analogToDegrees(servoPos)/180));

            dashTele.addData("Servo Angle: ", analogToDegrees(servoPos));
            dashTele.addData("Target Angle: ", target*180*mult);
            dashTele.addData("Servo Power: ", servo.getPower());
            dashTele.addData("Mult: ", mult);
            dashTele.addData("Elapsed Time: ", timer.elapsedTime());
            dashTele.addData("PIDF Values: ", "P: %.3f, I: %.3f, D: %.3f, F: %.3f", kP, kI, kD, A);
            dashTele.addData("PID Value smth: ", controller.getLastTimeStamp());
            dashTele.addData("PID Value smth2: ", controller.getVelocityError());
            dashTele.update();

            sleep(10);
        }
    }
}