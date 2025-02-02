package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.controllers.IIRSubsystem;

public class DistanceSensorsSubsystem extends SubsystemBase {
    /*
    * 0 -> Rear Sensor
    * 1 -> Left Sensor
    * 2 -> Right Sensor
    */

    private AnalogInput sensors[];
    private double[] distances = {0.0, 0.0, 0.0};
    private double[] filt_distances = {0.0, 0.0, 0.0};
    private IIRSubsystem iirSubsystem[];
    private double A = 0;

    private Telemetry telemetry;

    public DistanceSensorsSubsystem(RobotMap robotMap, Telemetry telemetry) {
        sensors = new AnalogInput[]{
                robotMap.getRearDist(),
                robotMap.getLeftDist(),
                robotMap.getRightDist()
        };

        iirSubsystem = new IIRSubsystem[]{
                new IIRSubsystem(A, ()->distances[0]),
                new IIRSubsystem(A, ()->distances[1]),
                new IIRSubsystem(A, ()->distances[2])
        };

        this.telemetry = telemetry;
    }

    public void periodic() {
        for (int i = 0; i < 3; i++) {
            distances[i] = (sensors[i].getVoltage()/3.3) * 500;
            filt_distances[i] = iirSubsystem[i].get();
        }

//        telemetry.addData("Rear Distance", distances[0]);
//        telemetry.addData("Left Distance", distances[1]);
//        telemetry.addData("Right Distance", distances[2]);
    }

    public double[] getDistances() {
        return distances;
    }

    public double[] getFilteredDistances() {
        return filt_distances;
    }
}
