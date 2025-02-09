package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.tipPoseTransfer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

@Autonomous(name = "AutoStarlight_v2", group = "Special")
public class AutoStarlight_v2 extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private DoubleSupplier extendo_length = ()-> 0.0;
    private OpCommon opCommon;
    private RobotMap robotMap;
    private SequentialCommandGroup temp;
    private Timing.Timer timer;

    /**
     * Poses
     */
    private Pose2d

        startPose = new Pose2d(
            -Tile + robotX/2, (-3 * Tile) + robotY/2, Math.toRadians(270)
        ),

        chambers = new Pose2d(
            -0.25 * Tile, -1.1 * Tile - (robotY/2), Math.toRadians(270)
        ),

        basket = new Pose2d(
            -2.55 * Tile, -2 * Tile, Math.toRadians(50)
        ),

        neutralSampleRight = new Pose2d(
            -2 * Tile - 2, -Tile - 2, Math.toRadians(90)
        ),

        neutralSampleMid = tipPoseTransfer(new Pose2d(
            -2.5 * Tile, -Tile, Math.toRadians(90)
        ), extendo_length.getAsDouble()),

        neutralSampleLeft = new Pose2d(
            -2.3 * Tile, -1.5 * Tile - 2, Math.toRadians(143)
        ),

        parking = new Pose2d(
            -1.25 * Tile, -0.5 * Tile, Math.toRadians(0)
        );

    /**
     * Trajectories
     */
    private TrajectorySequenceBuilder
        toPreload,
        toNeutral_0,
        toNeutral_1,
        toNeutral_2,
        toBasketExtra,
        toBasketFinal,
        toBasket_0,
        toBasket_1,
        toParking;

    public void init_toPreload() {
        toPreload = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .setTangent(Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(basket.getX() + 1, basket.getY(), Math.toRadians(75)), Math.toRadians(225));
    }
    public void init_toNeutral_0() {
        toNeutral_0 = drive.trajectorySequenceBuilder(current_pose)
                .lineToLinearHeading(new Pose2d(current_pose.getX() + 3, current_pose.getY(), Math.toRadians(60)));
    }
    public void init_toNeutral_1() {
        toNeutral_1 = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(new Pose2d(basket.getX(), basket.getY() + 5, Math.toRadians(85)));
    }
    public void init_toNeutral_2() {
        toNeutral_2 = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(neutralSampleLeft);
    }
    public void init_toBasket_0() {
        toBasket_0 = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .setTangent(Math.toRadians(315))
            .splineToSplineHeading(new Pose2d(basket.getX(), basket.getY() + 5,
                                             basket.getHeading()), Math.toRadians(180));
    }
    public void init_toBasketExtra() {
        toBasketExtra = drive.trajectorySequenceBuilder(current_pose)
                .setReversed(true)
                .setTangent(Math.toRadians(315))
                .splineToSplineHeading(new Pose2d(basket.getX(), basket.getY() + 5,
                        Math.toRadians(50)), Math.toRadians(180));
    }
    public void init_toBasket_1() {
        toBasket_1 = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .strafeTo(basket.vec());
    }
    public void init_toBasketFinal() {
        toBasketFinal = drive.trajectorySequenceBuilder(current_pose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(basket.getX(), basket.getY() + 5,
                        basket.getHeading()));
    }
    public void init_toParking() {
        toParking = drive.trajectorySequenceBuilder(current_pose)
            .splineTo(parking.vec(), Math.toRadians(0));
    }

    /**
     * La Program
     */
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.AUTO);
        opCommon = new OpCommon(robotMap);
        timer = new Timing.Timer(3000, TimeUnit.MILLISECONDS);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        temp = new SequentialCommandGroup(
                new WaitCommand(2000),
                opCommon.basket_scoring()
        );
        temp.schedule();
        init_toPreload();
        drive.followTrajectorySequenceAsync(toPreload.build());
        while (
            !isStopRequested()
                && opModeIsActive()
                && (drive.isBusy()
                || CommandScheduler.getInstance().isScheduled(temp))
        ) {
            drive.update();
            run();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = opCommon.release_sample();
        temp.schedule();
        while (
                !isStopRequested()
                        && opModeIsActive()
                        && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

        /* -----0----- */

        temp = new SequentialCommandGroup(
            opCommon.extendo(0.5),
            opCommon.sample_intake()
        );
        temp.schedule();
        init_toNeutral_0();
        drive.followTrajectorySequenceAsync(toNeutral_0.build());
        while (
            !isStopRequested()
            && opModeIsActive()
            && (drive.isBusy()
            || CommandScheduler.getInstance().isScheduled(temp))
        ) {
            drive.update();
            run();
        }
        current_pose = drive.getPoseEstimate();

        temp = opCommon.basket_scoring();
        temp.schedule();
        while (
            !isStopRequested()
            && opModeIsActive()
            && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

//        //distance sensor here
//        double value_1 = 5;
//        double value_2 = 5;
//
//        drive.setPoseEstimate(new Pose2d(
//            calculateReal2dLocation(current_pose, value_1, value_2), Math.toRadians(current_pose.getHeading())
//        ));
//        current_pose = drive.getPoseEstimate();

        temp = opCommon.release_sample();
        temp.schedule();
        while (
            !isStopRequested()
            && opModeIsActive()
            && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

        /* -----1----- */

        temp = new SequentialCommandGroup(
            new WaitCommand(500),
            opCommon.extendo(0.5),
            opCommon.sample_intake()
        );
        temp.schedule();
        init_toNeutral_1();
        drive.followTrajectorySequenceAsync(toNeutral_1.build());
        while (
            !isStopRequested()
            && opModeIsActive()
            && (drive.isBusy()
            || CommandScheduler.getInstance().isScheduled(temp))
        ) {
            drive.update();
            run();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = new SequentialCommandGroup(
                new WaitCommand(250),
                opCommon.basket_scoring()
        );
        temp.schedule();
        while (
            !isStopRequested()
                && opModeIsActive()
                && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

        basket = new Pose2d(basket.getX() - 2, basket.getY(), basket.getHeading());
        init_toBasket_1();
        drive.followTrajectorySequenceAsync(toBasket_1.build());
        while (
            !isStopRequested()
                && opModeIsActive()
                && drive.isBusy()
        ) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = opCommon.release_sample();
        temp.schedule();
        while (
            !isStopRequested()
            && opModeIsActive()
            && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

        /*-- 2 --*/
        temp = new SequentialCommandGroup(
            new WaitCommand(2000),
            opCommon.extendo3dSample(0.4),
            opCommon.sample_intake()
        );
        temp.schedule();
        init_toNeutral_2();
        drive.followTrajectorySequenceAsync(toNeutral_2.build());
        while (
            !isStopRequested()
            && opModeIsActive()
            && (drive.isBusy()
            || CommandScheduler.getInstance().isScheduled(temp))
        ) {
            drive.update();
            run();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = opCommon.basket_scoring();
        temp.schedule();
        basket = new Pose2d(basket.getX() + 5, basket.getY(), Math.toRadians(65));
        init_toBasketFinal();
        drive.followTrajectorySequenceAsync(toBasketFinal.build());
        while (
            !isStopRequested()
            && opModeIsActive()
            && drive.isBusy()
            && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            drive.update();
            run();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        //distance sensor here
//
//        drive.setPoseEstimate(new Pose2d(
//            calculateReal2dLocation(current_pose, value_1, value_2), Math.toRadians(current_pose.getHeading())
//        ));
//        current_pose = drive.getPoseEstimate();

        init_toBasket_1();
        drive.followTrajectorySequenceAsync(toBasket_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = opCommon.release_sample();
        temp.schedule();
        while (
            !isStopRequested()
                && opModeIsActive()
                && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

        /* -----P----- */

        temp = opCommon.reset_elevator();
        temp.schedule();
        init_toParking();
        drive.followTrajectorySequenceAsync(toParking.build());
        while (
                !isStopRequested()
                        && opModeIsActive()
                        && (drive.isBusy()
                        || CommandScheduler.getInstance().isScheduled(temp))
        ) {
            drive.update();
            run();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

//        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}