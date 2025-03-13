package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

//@Disabled
@TeleOp(name = "Limelight Test", group = "Tests")
public class LimelightTest extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100);
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        double[] inputs = {0, 35, 11, 3000, 0, 0, 640, 480};

        limelight.start();

//        isRed = 0
//        thresh = 35
//        blurAmount = 11
//        minContourArea = 3000
//        roi_x1 = 0
//        roi_y1 = 0
//        roi_x2 = 640
//        roi_y2 = 480

        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();

            if (status != null) {
                telemetry.addData("Name", "%s",
                        status.getName());
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(), (int) status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s",
                        status.getPipelineIndex(), status.getPipelineType());


//            limelight.updatePythonInputs(inputs);
                LLResult result = limelight.getLatestResult();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());

                double[] pythonOutputs = result.getPythonOutput();
                telemetry.addData("object x", pythonOutputs[0]);
                telemetry.addData("object y", pythonOutputs[1]);

                telemetry.update();
            }
        }

        limelight.stop();
    }
}
