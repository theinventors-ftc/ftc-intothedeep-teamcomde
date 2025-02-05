package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Auto.features.DistanceSensorLocalizer.calculateReal2dLocation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.DistanceSensorsSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;

@TeleOp(name = "SensorLocalizerTest", group = "Tests")
public class SensorLocalizerTest extends CommandOpMode {

    private RobotMap robotMap;
    private DistanceSensorsSubsystem distanceSensorsSubsystem;
    private SampleMecanumDrive drive;
    private Pose2d rrPose, realPose;
//    private FtcDashboard dashboard;
//    private Telemetry telemetry;
    private double[] distances;

    @Override
    public void initialize() {
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.TELEOP);
        distanceSensorsSubsystem = new DistanceSensorsSubsystem(this.robotMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-2 * 24, -2 * 24, Math.toRadians(90)));
    }

    @Override
    public void run() {
        super.run();

        drive.update();
        rrPose = drive.getPoseEstimate();
        distances = distanceSensorsSubsystem.getDistances();

        telemetry.addData("Distance Sensor Value Rear", distances[0]);
        telemetry.addData("Distance Sensor Value Left", distances[1]);

        realPose = new Pose2d(
            calculateReal2dLocation(
                rrPose,
                distances[0],
                distances[1]),
            drive.getPoseEstimate().getHeading());

        telemetry.addData("RR - X", rrPose.getX());
        telemetry.addData("RR - Y", rrPose.getY());
        telemetry.addData("RR - Theta", Math.toDegrees(rrPose.getHeading()));
        telemetry.addData("Distance - X", realPose.getX());
        telemetry.addData("Distance - Y", realPose.getY());
        telemetry.addData("Distance - Theta", Math.toDegrees(realPose.getHeading()));
        telemetry.update();
    }
}