package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.Features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.Features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.Features.BuilderFunctions.robotY;
import static org.firstinspires.ftc.teamcode.Auto.Features.BuilderFunctions.tipPoseTransfer;
import static org.firstinspires.ftc.teamcode.Auto.Features.DistanceSensorLocalizer.calculateReal2dLocation;
import static org.firstinspires.ftc.teamcode.Auto.Features.DistanceSensorLocalizer.calculateRealYLocation;
import static org.firstinspires.ftc.teamcode.Auto.opMode.OpCommon.fixedPose2d;
import static org.firstinspires.ftc.teamcode.Auto.opMode.OpCommon.init_mechanisms;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.DoubleSupplier;

@Autonomous(name = "Autonomous Left Red")
public class Red_Left_Samples extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private DoubleSupplier extedno_length;

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

        basket = new Pose2d(
            -2.5 * Tile, -2 * Tile, Math.toRadians(60)
        ),

        neutralSampleRight = tipPoseTransfer(new Pose2d(
            -2 * Tile - 2, -Tile, Math.toRadians(90)
        ), extedno_length.getAsDouble()),

        neutralSampleMid = tipPoseTransfer(new Pose2d(
            -2.5 * Tile, -Tile, Math.toRadians(90)
        ), extedno_length.getAsDouble()),

        neutralSampleLeft = tipPoseTransfer(new Pose2d(
            -3 * Tile + 3, -Tile, Math.toRadians(90)
        ), extedno_length.getAsDouble()),

        parking = tipPoseTransfer(new Pose2d(
            -Tile, -0.5 * Tile, Math.toRadians(45)
        ), extedno_length.getAsDouble());

    /**
     * Trajectories
     */
    private TrajectorySequenceBuilder
        toPreload_0,
        toPreload_1,
        toNeutral_0,
        toNeutral_1,
        toNeutral_2,
        toBasket_0,
        toBasket_1,
        toParking;

    public void init_toPreload_0() {
        toPreload_0 = drive.trajectorySequenceBuilder(startPose)
            .strafeTo(new Vector2d(chambers.getX(), chambers.getY() - 5));
    }
    public void init_toPreload_1() {
        toPreload_1 = drive.trajectorySequenceBuilder(current_pose)
            .strafeTo(chambers.vec());
    }
    public void init_toNeutral_0() {
        toNeutral_0 = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(225)
            .splineToSplineHeading(neutralSampleRight, Math.toRadians(135));
    }
    public void init_toNeutral_1() {
        toNeutral_1 = drive.trajectorySequenceBuilder(current_pose)
            .lineToSplineHeading(neutralSampleMid);
    }
    public void init_toNeutral_2() {
        toNeutral_2 = drive.trajectorySequenceBuilder(current_pose)
            .lineToSplineHeading(neutralSampleLeft);
    }
    public void init_toBasket_0() {
        toBasket_0 = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .splineTo(new Vector2d(basket.getX(), basket.getY() + 5), basket.getHeading());
    }
    public void init_toBasket_1() {
        toBasket_1 = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .strafeTo(basket.vec());
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

        //mechanisms

        /* -----0----- */

        init_toNeutral_0();
        drive.followTrajectorySequenceAsync(toNeutral_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //parallel mechanisms

        init_toBasket_0();
        drive.followTrajectorySequenceAsync(toBasket_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //distance sensor here
        double value_1 = 5;
        double value_2 = 5;

        drive.setPoseEstimate(new Pose2d(
            calculateReal2dLocation(current_pose, value_1, value_2), Math.toRadians(current_pose.getHeading())
        ));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        init_toBasket_1();
        drive.followTrajectorySequenceAsync(toBasket_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        /* -----1----- */

        init_toNeutral_1();
        drive.followTrajectorySequenceAsync(toNeutral_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //parallel mechanisms

        init_toBasket_0();
        drive.followTrajectorySequenceAsync(toBasket_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //distance sensor here

        drive.setPoseEstimate(new Pose2d(
            calculateReal2dLocation(current_pose, value_1, value_2), Math.toRadians(current_pose.getHeading())
        ));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        init_toBasket_1();
        drive.followTrajectorySequenceAsync(toBasket_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        /* -----2----- */

        init_toNeutral_2();
        drive.followTrajectorySequenceAsync(toNeutral_2.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //parallel mechanisms

        init_toBasket_0();
        drive.followTrajectorySequenceAsync(toBasket_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //distance sensor here

        drive.setPoseEstimate(new Pose2d(
            calculateReal2dLocation(current_pose, value_1, value_2), Math.toRadians(current_pose.getHeading())
        ));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        init_toBasket_1();
        drive.followTrajectorySequenceAsync(toBasket_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        /* -----P----- */

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