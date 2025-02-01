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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.function.DoubleSupplier;

@Autonomous(name = "Red Left Samples ONLY 2", group = "RED")
public class Red_Left_Samples_2_ONLY extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private DoubleSupplier extedno_length = ()-> 0.0;
    private OpCommon opCommon;
    private RobotMap robotMap;
    private SequentialCommandGroup temp;

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
            -2.55 * Tile, -2 * Tile + 1, Math.toRadians(75)
        ),

        neutralSampleRight = new Pose2d(
            -2 * Tile - 2, -Tile - 2, Math.toRadians(90)
        ),

        neutralSampleMid = tipPoseTransfer(new Pose2d(
            -2.5 * Tile, -Tile, Math.toRadians(90)
        ), extedno_length.getAsDouble()),

        neutralSampleLeft = new Pose2d(
            -2.5 * Tile, -Tile - 5, Math.toRadians(180)
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
        toBasket_0,
        toBasket_1,
        toParking;

    public void init_toPreload() {
        toPreload = drive.trajectorySequenceBuilder(startPose)
            .waitSeconds(0.5)
            .strafeTo(chambers.vec());
    }
    public void init_toNeutral_0() {
        toNeutral_0 = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(Math.toRadians(270))
            .splineTo(neutralSampleRight.vec(), Math.toRadians(135),
                      SampleMecanumDrive.getVelocityConstraint(
                          DriveConstants.MAX_VEL,
                          DriveConstants.MAX_ANG_VEL,
                          DriveConstants.TRACK_WIDTH
                      ),
                      SampleMecanumDrive.getAccelerationConstraint(20));
    }
    public void init_toNeutral_1() {
        toNeutral_1 = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(new Pose2d(basket.getX(), basket.getY() + 5, Math.toRadians(85)));
    }
    public void init_toNeutral_2() {
        toNeutral_2 = drive.trajectorySequenceBuilder(current_pose)
            .splineTo(new Vector2d(-1.8 * Tile, -1.5 * Tile), Math.toRadians(90))
            .splineToSplineHeading(neutralSampleLeft, Math.toRadians(180));
    }
    public void init_toBasket_0() {
        toBasket_0 = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .setTangent(Math.toRadians(315))
            .splineToSplineHeading(new Pose2d(basket.getX(), basket.getY() + 5,
                                             basket.getHeading()), Math.toRadians(180));
    }
    public void init_toBasket_1() {
        toBasket_1 = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .strafeTo(basket.vec());
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
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, true);
        opCommon = new OpCommon(robotMap);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        temp = opCommon.raise_high_chamber();
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

        /* -----0----- */

        temp = new SequentialCommandGroup(
            new WaitCommand(1000),
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
        init_toBasket_0();
        drive.followTrajectorySequenceAsync(toBasket_0.build());
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

//        //distance sensor here
//        double value_1 = 5;
//        double value_2 = 5;
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

        /* -----1----- */

        temp = new SequentialCommandGroup(
            opCommon.extendo(),
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

        temp = opCommon.basket_scoring();
        temp.schedule();
        while (
            !isStopRequested()
                && opModeIsActive()
                && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

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