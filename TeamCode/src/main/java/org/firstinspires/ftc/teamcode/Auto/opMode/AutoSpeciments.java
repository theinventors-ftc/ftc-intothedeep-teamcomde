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
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.trajectory.constraint.TrajectoryConstraint;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Auto.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

@Autonomous(name = "AutoSpeciments")
public class AutoSpeciments extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private OpCommon opCommon;
    private RobotMap robotMap;
    private SequentialCommandGroup temp;
    private Timing.Timer timer;

    private final double[] pidaTargets = {40, 66, 90};

    private final double
        transPIDTolerance = 1,
        headPIDTolerance = 2;

    /**
     * Poses
     */
    private Pose2d

        startPose = new Pose2d(
            Tile - robotX/2, (-3 * Tile) + robotY/2 + 2.5, Math.toRadians(90)
        ),

        preload = new Pose2d(
            3 , -Tile - (robotY/2) + 1.5, Math.toRadians(90)
        ),

        chambers = new Pose2d(
            0 , -Tile - (robotY/2) + 1, Math.toRadians(90)
        ),

        observationZone = new Pose2d(
            1.5 * Tile, -3 * Tile + (robotY/2) + 17, Math.toRadians(90)
        ),

        allianceSampleLeft = new Pose2d(
            2 * Tile + 2, -Tile + (robotY/2) + 3, Math.toRadians(90)
        ),

        allianceSampleMid = new Pose2d(
            2.5 * Tile - 2, -Tile + (robotY/2) + 1, Math.toRadians(90)
        ),

        allianceSampleRight = new Pose2d(
            2.5 * Tile, -Tile/2, Math.toRadians(90)
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
        toHumanPlayer,
        toScoreSpeciment,
        smallLeft,
        toParking;

    public void init_toPreload() {
        toPreload = drive.trajectorySequenceBuilder(startPose)
            .waitSeconds(0.3)
            .strafeTo(preload.vec(),
                      SampleMecanumDrive.getVelocityConstraint(60,
                                                               DriveConstants.MAX_ANG_VEL,
                                                               DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(45)
            );
    }

    public void init_toAllianceSamples() {
        toAllianceSamples = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(Math.toRadians(315))
            .splineToConstantHeading(new Vector2d(Tile + 5, -1.7 * Tile), Math.toRadians(45))
            .splineToConstantHeading(allianceSampleLeft.vec(), Math.toRadians(270))
            .lineToConstantHeading(new Vector2d(allianceSampleLeft.getX(),
                                                (-2.3 * Tile) + (robotY/2)),
                                   SampleMecanumDrive.getVelocityConstraint(60,
                                                                            DriveConstants.MAX_ANG_VEL,
                                                                            DriveConstants.TRACK_WIDTH),
                                   SampleMecanumDrive.getAccelerationConstraint(55)
            )
            .lineToConstantHeading(allianceSampleLeft.vec(),
                                   SampleMecanumDrive.getVelocityConstraint(60,
                                                                            DriveConstants.MAX_ANG_VEL,
                                                                            DriveConstants.TRACK_WIDTH),
                                   SampleMecanumDrive.getAccelerationConstraint(55)
            )
            .splineToConstantHeading(allianceSampleMid.vec(), Math.toRadians(0))
            .lineToConstantHeading(new Vector2d(allianceSampleMid.getX(),
                                                (-2.4 * Tile) + (robotY/2)),
                                   SampleMecanumDrive.getVelocityConstraint(60,
                                                                            DriveConstants.MAX_ANG_VEL,
                                                                            DriveConstants.TRACK_WIDTH),
                                   SampleMecanumDrive.getAccelerationConstraint(45)
            );

//            .strafeTo(new Vector2d(allianceSampleLeft.getX(), (-2.5 * Tile) + (robotY/2) + 15))
//            .strafeTo(new Vector2d(allianceSampleMid.getX() + 1.5, (-2.1 * Tile) + (robotY/2)));
    }

    public void init_toHumanPlayer() {
        toHumanPlayer = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .lineToConstantHeading(observationZone.vec());
    }

    public void init_toObservationZone() {
        toObservationZone = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .setTangent(Math.toRadians(315))
            .splineToConstantHeading(observationZone.vec(), Math.toRadians(270));
    }

//    public void init_smallLeft() {
//        smallLeft = drive.trajectorySequenceBuilder(current_pose)
//            .strafeLeft(3);
//    }

    public void init_toScoreSpeciment() {
        toScoreSpeciment = drive.trajectorySequenceBuilder(current_pose)
//            .lineToLinearHeading(chambers,
//                                 SampleMecanumDrive.getVelocityConstraint(60,
//                                                                            DriveConstants.MAX_ANG_VEL,
//                                                                            DriveConstants.TRACK_WIDTH),
//                                 SampleMecanumDrive.getAccelerationConstraint(55))
            .setTangent(135)
            .splineToSplineHeading(chambers, Math.toRadians(90),
                      SampleMecanumDrive.getVelocityConstraint(60,
                                                               DriveConstants.MAX_ANG_VEL,
                                                               DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(55))
            .forward(2);
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
        opCommon.init_controllers(drive);
        timer = new Timing.Timer(1200, TimeUnit.MILLISECONDS);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        temp = opCommon.specimenIntake();
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
        drive.setPoseEstimate(new Pose2d(
            drive.getPoseEstimate().getX(),
            -Tile - (robotY/2) + 1.5,
            Math.toRadians(90))
        );
        current_pose = drive.getPoseEstimate();

        temp = new SequentialCommandGroup(
            opCommon.scoreSpeciment(),
            opCommon.releaseSpecimen()
        );
        temp.schedule();
        while (
            !isStopRequested()
                && opModeIsActive()
                && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

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
        current_pose = drive.getPoseEstimate();

        temp = new SequentialCommandGroup(
            new WaitCommand(100),
            opCommon.specimenAim()
        );
        temp.schedule();
        init_toHumanPlayer();
        drive.followTrajectorySequenceAsync(toHumanPlayer.build());
        while (
            !isStopRequested()
                && opModeIsActive()
                && (drive.isBusy()
                || CommandScheduler.getInstance().isScheduled(temp))
        ) {
            drive.update();
            run();
        }
        //            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        for(int i = 0; i < 3; i++) {

            /* -- PID Correction -- */
            drive.deactivate();

            temp = opCommon.activateDistanceCalibration(pidaTargets);
            temp.schedule();
            timer.start();
            while (
                !isStopRequested()
                    && !timer.done()
                    && opModeIsActive()
                    && !opCommon.isInThreshold(
                        opCommon.distanceSensorsSubsystem.getDistances()[0],
                        pidaTargets[0],
                        transPIDTolerance
                )
                    && !opCommon.isInThreshold(
                        opCommon.distanceSensorsSubsystem.getDistances()[2],
                        pidaTargets[1],
                        transPIDTolerance)

                    && !opCommon.isInThreshold(
                        -opCommon.gyroFollow.calculateTurn(),
                        pidaTargets[2],
                        headPIDTolerance
                )
            ) {
                opCommon.robotCentricMovement(
                    opCommon.drivetrainStrafe(),
                    opCommon.drivetrainForward(),
                    opCommon.drivetrainTurn());

                drive.update();
                run();
            }

            temp = opCommon.deactivateDistanceCalibration();
            temp.schedule();
            while (
                !isStopRequested()
                    && opModeIsActive()
                    && CommandScheduler.getInstance().isScheduled(temp)
            ) {
                run();
            }

            drive.setPoseEstimate(new Pose2d(
                1.5 * Tile, -3 * Tile + (robotY/2) + 15.5 + 0.75, Math.toRadians(90)
            ));
            drive.activate();
//            /* -- PID Correction -- */

            temp = opCommon.specimenIntake();
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

            chambers = new Pose2d(chambers.getX() - 2, chambers.getY(), chambers.getHeading());

            drive.setPoseEstimate(new Pose2d(
                drive.getPoseEstimate().getX(),
                -Tile - (robotY/2) + 1.5,
                Math.toRadians(90))
            );
            current_pose = drive.getPoseEstimate();

            temp = opCommon.scoreSpeciment();
            temp.schedule();
            while (
                !isStopRequested()
                    && opModeIsActive()
                    && CommandScheduler.getInstance().isScheduled(temp)
            ) {
                run();
            }

//            init_smallLeft();
//            drive.followTrajectorySequenceAsync(smallLeft.build());
//            while(
//                !isStopRequested()
//                    && opModeIsActive()
//                    && drive.isBusy()
//            ) {
//                drive.update();
//            }

            temp = opCommon.releaseSpecimen();
            temp.schedule();
            while (
                !isStopRequested()
                    && opModeIsActive()
                    && CommandScheduler.getInstance().isScheduled(temp)
            ) {
                run();
            }

            temp = new SequentialCommandGroup(
                new WaitCommand(100),
                opCommon.specimenAim()
            );
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
            //            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            current_pose = drive.getPoseEstimate();
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
