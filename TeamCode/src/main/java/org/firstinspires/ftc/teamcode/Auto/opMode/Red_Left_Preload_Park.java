package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.tipPoseTransfer;
import static org.firstinspires.ftc.teamcode.Auto.features.DistanceSensorLocalizer.calculateRealYLocation;
import static org.firstinspires.ftc.teamcode.Auto.opMode.OpCommon.init_mechanisms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.DoubleSupplier;

@Autonomous(name = "Autonomous Left Red Preload and Park")
public class Red_Left_Preload_Park extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private Double extedno_length = 0.0;

    /**
     * Poses
     */
    private Pose2d

        startPose = new Pose2d(
            -Tile + robotX/2, (-3 * Tile) + robotY/2, Math.toRadians(270)
        ),

        chambers = new Pose2d(
            -0.25 * Tile, -Tile - (robotY/2), Math.toRadians(270)
        ),

        parking = new Pose2d(
            -Tile, -0.5 * Tile, Math.toRadians(0)
        );

    /**
     * Trajectories
     */
    private TrajectorySequenceBuilder
        toPreload_0,
        toPreload_1,
        toParking;

    public void init_toPreload_0() {
        toPreload_0 = drive.trajectorySequenceBuilder(startPose)
            .strafeTo(new Vector2d(chambers.getX(), chambers.getY() - 5));
    }
    public void init_toPreload_1() {
        toPreload_1 = drive.trajectorySequenceBuilder(current_pose)
            .strafeTo(chambers.vec());
    }
    public void init_toParking() {
        toParking = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(Math.toRadians(270))
            .splineTo(new Vector2d(-1.5 * Tile, -1.5 * Tile), Math.toRadians(135))
            .splineTo(parking.vec(), Math.toRadians(0));
    }

    /**
     * La Program
     */
    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        init_mechanisms(hardwareMap, telemetry);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        init_toPreload_0();
        drive.followTrajectorySequenceAsync(toPreload_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

//        //distance sensor here
//        double value = 5;
//
//        drive.setPoseEstimate(new Pose2d(
//            calculateRealYLocation(current_pose, value), Math.toRadians(current_pose.getHeading())
//        ));
//        current_pose = drive.getPoseEstimate();

        init_toPreload_1();
        drive.followTrajectorySequenceAsync(toPreload_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        //mechanisms ktlp

        init_toParking();
        drive.followTrajectorySequenceAsync(toParking.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

//        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}