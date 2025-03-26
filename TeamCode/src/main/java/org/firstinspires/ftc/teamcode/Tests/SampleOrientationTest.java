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
@TeleOp(name = "Sample Orientation Test", group = "Tests")
public class SampleOrientationTest extends LinearOpMode {
    private Limelight3A limelight;
    private LLResult result;

    private IIRSubsystem angle;
    private double angle_dec = 0.0;
    public static double A = 0.94, DEC = 8.0;
    private Telemetry dashTele;
    private double servoValue = 0.0;
    private ServoImplEx clawServo;

    public double mapping(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x-in_min) * (out_max-out_min) / (in_max - in_min) + out_min;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        clawServo = hardwareMap.get(ServoImplEx.class, "servo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        dashTele = FtcDashboard.getInstance().getTelemetry();
        dashTele.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        angle = new IIRSubsystem(A, () -> angle_dec);

        waitForStart();

        while (opModeIsActive()) {
            result = limelight.getLatestResult();
            angle.set(A);
            if (result != null) {
                dashTele.addData("Sample X:", result.getPythonOutput()[0]);
                dashTele.addData("Sample Y:", result.getPythonOutput()[1]);
                dashTele.addData("Sample RX:", result.getPythonOutput()[2]);

                angle_dec = ((int)(result.getPythonOutput()[2]/DEC)) * DEC;

                angle.periodic();

                servoValue = mapping(angle.get(), 0, 180, 0.0, 1.0);

                clawServo.setPosition(servoValue);

                dashTele.addData("Servo Value (Unfilt):", angle_dec);
                dashTele.addData("Servo Value:", angle.get());
            } else {
                dashTele.addData("Limelight", "No data available");
            }
            dashTele.update();
        }
    }
}
