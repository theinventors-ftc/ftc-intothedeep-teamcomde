package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.tipPoseTransfer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.DoubleSupplier;

@Disabled
@Autonomous(name = "Autonomous Left Blue")
public class Blue_Left extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private DoubleSupplier extendo_length;

    /**
     * Poses
     */
    private Pose2d

        startPose = new Pose2d(
            Tile - robotX/2, (3 * Tile) - robotY/2, Math.toRadians(90)
        ),

        chambers = new Pose2d(
            0.25 * Tile, Tile + (robotY/2), Math.toRadians(90)
        ),

        submersibleSide = new Pose2d(
            Tile + (robotY/2), Tile/2, Math.toRadians(180)
        ),

        basket = new Pose2d(
            2.5 * Tile, 2 * Tile, Math.toRadians(240)
        ),

        neutralSampleRight = tipPoseTransfer(new Pose2d(
            2 * Tile + 2, Tile, Math.toRadians(270)
        ), extendo_length.getAsDouble()),

        neutralSampleMid = tipPoseTransfer(new Pose2d(
            2.5 * Tile, Tile, Math.toRadians(270)
        ), extendo_length.getAsDouble()),

        neutralSampleLeft = tipPoseTransfer(new Pose2d(
            3 * Tile - 3, Tile, Math.toRadians(270)
        ), extendo_length.getAsDouble()),

        parking = tipPoseTransfer(new Pose2d(
            Tile, 0.5 * Tile, Math.toRadians(225)
        ), extendo_length.getAsDouble());

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
        toSubmersibleSide,
        toParking;

    public void init_toPreload_0() {
        toPreload_0 = drive.trajectorySequenceBuilder(startPose)
            .strafeTo(new Vector2d(chambers.getX(), chambers.getY() + 5));
    }
    public void init_toPreload_1() {
        toPreload_1 = drive.trajectorySequenceBuilder(current_pose)
            .strafeTo(chambers.vec());
    }
    public void init_toNeutral_0() {
        toNeutral_0 = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(45)
            .splineToSplineHeading(neutralSampleRight, Math.toRadians(315));
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
            .splineTo(new Vector2d(basket.getX(), basket.getY() - 5), basket.getHeading());
    }
    public void init_toBasket_1() {
        toBasket_1 = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .strafeTo(basket.vec());
    }
    public void init_toSubmersibleSide() {
        toSubmersibleSide = drive.trajectorySequenceBuilder(current_pose)
            .splineTo(submersibleSide.vec(), Math.toRadians(submersibleSide.getHeading()));
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
    }

    @Override
    public void run() {

        init_toNeutral_0();
        drive.followTrajectorySequenceAsync(toNeutral_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        //        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
