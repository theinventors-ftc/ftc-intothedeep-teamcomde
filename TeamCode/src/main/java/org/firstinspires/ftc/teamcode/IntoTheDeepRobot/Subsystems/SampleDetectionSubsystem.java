package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.controllers.IIRSubsystem;

public class SampleDetectionSubsystem extends SubsystemBase {
    private boolean enabled = false;
    private Limelight3A limelight;
    private LLResult result;
    private IIRSubsystem angle;
    private double angle_dec = 0.0;
    public static double A = 0.94, DEC = 8.0;
    private Telemetry telemetry;

    public SampleDetectionSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.telemetry = telemetry;
//        limelight = robotMap.getRearLimelight();
        limelight = hm.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        angle = new IIRSubsystem(A, () -> angle_dec);
    }

    @Override
    public void periodic() {
        if(enabled) {
            result = limelight.getLatestResult();
            telemetry.addData("Limelight", result.getPythonOutput()[2]);
            angle_dec = ((int)(result.getPythonOutput()[2]/DEC)) * DEC;
        }
    }

    public double getAngle() {
        double cur_angle = this.angle.get();

        if (cur_angle > 90 && cur_angle < 270) {
            cur_angle -= 3180;
        } else if (cur_angle > 270) {
            cur_angle -= 360;
            cur_angle %= 360;
        }
        return cur_angle;
    }

    // ------------------------------------------ States ---------------------------------------- //

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void enable() {
        setEnabled(true);
    }

    public void disable() {
        setEnabled(false);
    }
}
