package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Auto.features.DistanceSensorLocalizer.calculateReal2dLocation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.DistanceSensorsSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;
@Disabled
@TeleOp(name = "LocalizationTest", group = "Tests")
public class LocalizationTest extends CommandOpMode {

    private SampleMecanumDrive drive;
    private Pose2d rrPose, realPose;
    private RobotMap robotMap;

    private IntakeSubsystem in;
    private DistanceSensorsSubsystem dist;

    @Override
    public void initialize() {
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.TELEOP);
        dist = new DistanceSensorsSubsystem(robotMap, telemetry);
        drive = new SampleMecanumDrive(robotMap);
        drive.setPoseEstimate(new Pose2d(0,0,0));
        in = new IntakeSubsystem(robotMap);
    }

    @Override
    public void run() {
        super.run();

        drive.update();
        rrPose = drive.getPoseEstimate();

        telemetry.addData("RR - X", rrPose.getX());
        telemetry.addData("RR - Y", rrPose.getY());
        telemetry.addData("RR - Theta", Math.toDegrees(rrPose.getHeading()));
        telemetry.addData("X", dist.getDistances()[0]);
        telemetry.addData("Y", dist.getDistances()[2]);
        telemetry.update();
    }
}
