package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Auto.features.DistanceSensorLocalizer.calculateReal2dLocation;

import android.service.quicksettings.Tile;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.features.DistanceSensorLocalizer;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.DistanceSensorsSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;

@TeleOp(name = "SensorLocalizerTest", group = "Tests")
public class SensorLocalizerTest extends LinearOpMode {

    private RobotMap robotMap;
    private DistanceSensorsSubsystem distanceSensorsSubsystem;
    private SampleMecanumDrive drive;
    private Pose2d realPose = new Pose2d();

    @Override
    public void runOpMode() {
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, false);
        distanceSensorsSubsystem = new DistanceSensorsSubsystem(robotMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-2 * 24, -2 * 24, Math.toRadians(90)));
        waitForStart();

        while(isStopRequested() && !opModeIsActive()) {
            drive.update();
            realPose = new Pose2d(calculateReal2dLocation(
                drive.getPoseEstimate(),
                distanceSensorsSubsystem.getDistances()[0],
                distanceSensorsSubsystem.getDistances()[1]),
                                  drive.getPoseEstimate().getHeading());
            telemetry.addData("X", realPose.getX());
            telemetry.addData("Y", realPose.getY());
            telemetry.addData("Theta", Math.toDegrees(realPose.getHeading()));
            telemetry.update();

        }
    }
}
