package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.tipPoseTransfer;
import static org.firstinspires.ftc.teamcode.Auto.features.DistanceSensorLocalizer.calculateRealYLocation;
import static org.firstinspires.ftc.teamcode.Auto.opMode.OpCommon.fixedPose2d;
import static org.firstinspires.ftc.teamcode.Auto.opMode.OpCommon.init_mechanisms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.DoubleSupplier;

@Autonomous(name = "Autonomous Right Red Preload and Park")
public class Red_Right_Preload_Park extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private DoubleSupplier extedno_length;

    /**
     * Poses
     */
    private Pose2d

        startPose = new Pose2d(
            Tile - robotX/2, (-3 * Tile) + robotY/2, Math.toRadians(270)
        ),

        preload = new Pose2d(
            0.25 * Tile , -Tile - (robotY/2), Math.toRadians(270)
        ),

        parking = tipPoseTransfer(new Pose2d(
            2 * Tile, -2.75 * Tile, Math.toRadians(315)
        ), extedno_length.getAsDouble());

    /**
     * Trajectories
     */
    private TrajectorySequenceBuilder
        toPreload_0,
        toPreload_1,
        toParking;

    public void init_toPreload_0() {
        toPreload_0 = drive.trajectorySequenceBuilder(startPose)
            .strafeTo(new Vector2d(preload.getX(), preload.getY() - 5));
    }
    public void init_toPreload_1() {
        toPreload_1 = drive.trajectorySequenceBuilder(current_pose)
            .strafeTo(preload.vec());
    }
    public void init_toParking() {
        toParking = drive.trajectorySequenceBuilder(current_pose)
            .splineTo(parking.vec(), Math.toRadians(parking.getHeading()));
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
    public void run() {

        init_toPreload_0();
        drive.followTrajectorySequenceAsync(toPreload_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //distance sensor here
        double value = 5;

        drive.setPoseEstimate(new Pose2d(
            calculateRealYLocation(current_pose, value), Math.toRadians(current_pose.getHeading())
        ));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        init_toPreload_1();
        drive.followTrajectorySequenceAsync(toPreload_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms ktlp

        init_toParking();
        drive.followTrajectorySequenceAsync(toParking.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

//        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
