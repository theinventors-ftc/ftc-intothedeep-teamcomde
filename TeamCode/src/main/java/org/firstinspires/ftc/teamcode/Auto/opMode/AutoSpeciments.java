package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.tipPoseTransfer;
import static org.firstinspires.ftc.teamcode.Auto.features.DistanceSensorLocalizer.calculateReal2dLocation;
import static org.firstinspires.ftc.teamcode.Auto.features.DistanceSensorLocalizer.calculateRealYLocation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;

import java.util.function.DoubleSupplier;

@Disabled
@Autonomous(name = "AutoSpeciments")
public class AutoSpeciments extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private OpCommon opCommon;
    private RobotMap robotMap;
    private SequentialCommandGroup temp;

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

        chambers = new Pose2d(
            0 , -Tile - (robotY/2), Math.toRadians(270)
        ),

        observationZone = new Pose2d(
            1.5 * Tile, -2.5 * Tile, Math.toRadians(90)
        ),

        allianceSampleLeft = new Pose2d(
            2 * Tile, -Tile/2, Math.toRadians(90)
        ),

        allianceSampleMid = new Pose2d(
            2.5 * Tile, -Tile/2, Math.toRadians(90)
        ),

        allianceSampleRight = new Pose2d(
            3 * Tile - 3, -Tile/2, Math.toRadians(90)
        ),

        parking = new Pose2d(
            2 * Tile, -2.75 * Tile, Math.toRadians(315)
        );

    /**
     * Trajectories
     */
    private TrajectorySequenceBuilder
        toPreload,
        toAllianceSamples,
        toObservationZone,
        toScoreSpeciment,
        toParking;

    public void init_toPreload() {
        toPreload = drive.trajectorySequenceBuilder(startPose)
            .waitSeconds(0.5)
            .strafeTo(preload.vec());
    }

    public void init_toAllianceSamples() {
        toAllianceSamples = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(315)
            .splineToSplineHeading(allianceSampleLeft, Math.toRadians(0))
            .lineToConstantHeading(new Vector2d(current_pose.getX(), -2.6 * Tile))
            .lineToLinearHeading(allianceSampleLeft)
            .splineToConstantHeading(allianceSampleMid.vec(), Math.toRadians(0))
            .lineToConstantHeading(new Vector2d(current_pose.getX(), -2.6 * Tile))
            .lineToLinearHeading(allianceSampleMid)
            .splineToConstantHeading(allianceSampleRight.vec(), Math.toRadians(0))
            .lineToConstantHeading(new Vector2d(current_pose.getX(), -2.6 * Tile));

    }

    public void init_toObservationZone() {
        toObservationZone = drive.trajectorySequenceBuilder(current_pose)
            .strafeLeft(2)
            .setReversed(true)
            .lineToConstantHeading(observationZone.vec());
    }

    public void init_toScoreSpeciment() {
        toScoreSpeciment = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(chambers)
            .forward(3);
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
        CommandScheduler.getInstance().reset();
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.AUTO);
        drive = new SampleMecanumDrive(robotMap);
        drive.setPoseEstimate(startPose);
        opCommon = new OpCommon(robotMap, RobotEx.Alliance.RED);
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


        temp = opCommon.reset_elevator();
        temp.schedule();
        init_toAllianceSamples();
        drive.followTrajectorySequenceAsync(toAllianceSamples.build());
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


        for(int i = 0; i < 3; i++) {

            temp = opCommon.reset_elevator();
            temp.schedule();
            init_toObservationZone();
            drive.followTrajectorySequenceAsync(toObservationZone.build());
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

            temp = opCommon.reset_elevator();
            temp.schedule();
            init_toScoreSpeciment();
            drive.followTrajectorySequenceAsync(toScoreSpeciment.build());
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

            current_pose = new Pose2d(current_pose.getX(),
                                      current_pose.getY(),
                                      Math.toRadians(270));
        }

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
