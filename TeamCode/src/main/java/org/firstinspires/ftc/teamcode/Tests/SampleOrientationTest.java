package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

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
    private double servoValue = 0.0;
    private ServoImplEx clawServo;
    double[] inputs = {0.0, 30.0, 11.0, 3000.0, 0.0, 0.0, 640.0, 480.0};

    @Override
    public void runOpMode() throws InterruptedException {
        clawServo = hardwareMap.get(ServoImplEx.class, "claw_rot");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        angle = new IIRSubsystem(A, () -> angle_dec);

        waitForStart();

        while (opModeIsActive()) {
            limelight.updatePythonInputs(inputs);
            result = limelight.getLatestResult();
            angle.set(A);
            if (result != null) {
                telemetry.addData("Sample X:", result.getPythonOutput()[0]);
                telemetry.addData("Sample Y:", result.getPythonOutput()[1]);
                telemetry.addData("Sample RX:", result.getPythonOutput()[2]);

                angle_dec = ((int)(result.getPythonOutput()[2]/DEC)) * DEC;

                angle.periodic();

                servoValue = Range.scale(angle.get(), 0, 180, 0.0, 1.0);

                clawServo.setPosition(servoValue);

                telemetry.addData("Servo Value (Unfilt):", angle_dec);
                telemetry.addData("Servo Value:", angle.get());
            } else {
                telemetry.addData("Limelight", "No data available");
            }
            telemetry.update();
        }
    }
}
