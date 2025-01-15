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

@Autonomous(name = "Autonomous Right Blue Speciments")
public class Blue_Right_Speciments extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private DoubleSupplier extedno_length;

    /**
     * Poses
     */
    private Pose2d

        startPose = new Pose2d(
            -Tile + robotX/2, (3 * Tile) - robotY/2, Math.toRadians(90)
        ),

        preload = new Pose2d(
            -0.25 * Tile , Tile + (robotY/2), Math.toRadians(90)
        ),

        chambers = new Pose2d(
            0 , Tile + (robotY/2), Math.toRadians(90)
        ),

        observationZone = new Pose2d(
            -2 * Tile, 3 * Tile - (robotY/2) - 3, Math.toRadians(270)
        ),

        submersibleSide = new Pose2d(
            -Tile - (robotY/2), Tile/2, Math.toRadians(0)
        ),

        allianceSampleLeft = tipPoseTransfer(new Pose2d(
            -2 * Tile - 2, Tile, Math.toRadians(225)
        ), extedno_length.getAsDouble()),

        allianceSampleMid = tipPoseTransfer(new Pose2d(
            -2.5 * Tile, Tile, Math.toRadians(225)
        ), extedno_length.getAsDouble()),

        allianceSampleRight = tipPoseTransfer(new Pose2d(
            -3 * Tile + 3, Tile, Math.toRadians(225)
        ), extedno_length.getAsDouble()),

        parking = tipPoseTransfer(new Pose2d(
            -2 * Tile, 2.75 * Tile, Math.toRadians(135)
        ), extedno_length.getAsDouble());

    /**
     * Trajectories
     */
    private TrajectorySequenceBuilder
        toPreload_0,
        toPreload_1,
        toAlliance_0,
        toAlliance_1,
        toAlliance_2,
        turn,
        toSubmersibleSide,
        toObservationZone_0,
        toObservationZone_1,
        toScoreSpeciment_0,
        toScoreSpeciment_1,
        toParking;

    public void init_toPreload_0() {
        toPreload_0 = drive.trajectorySequenceBuilder(startPose)
            .strafeTo(new Vector2d(preload.getX(), preload.getY() + 5));
    }
    public void init_toPreload_1() {
        toPreload_1 = drive.trajectorySequenceBuilder(current_pose)
            .strafeTo(preload.vec());
    }
    public void init_toAlliance_0() {
        toAlliance_0 = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(135)
            .splineToSplineHeading(allianceSampleLeft, Math.toRadians(240));
    }
    public void init_toAlliance_1() {
        toAlliance_1 = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(allianceSampleMid);
    }
    public void init_toAlliance_2() {
        toAlliance_2 = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(allianceSampleRight);
    }
    public void init_turn() {
        turn = drive.trajectorySequenceBuilder(current_pose)
            .turn(Math.toRadians(-90));
    }
    public void init_toObservationZone_0() {
        toObservationZone_0 = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(90)
            .splineToSplineHeading(new Pose2d(observationZone.getX(),
                                              observationZone.getY() - 3,
                                              Math.toRadians(observationZone.getHeading())),
                                   Math.toRadians(90));
    }
    public void init_toObservationZone_1() {
        toObservationZone_1 = drive.trajectorySequenceBuilder(current_pose)
            .strafeTo(observationZone.vec());
    }
    public void init_toScoreSpeciment_0() {
        toScoreSpeciment_0 = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(315)
            .splineToSplineHeading(new Pose2d(chambers.getX(),
                                              chambers.getY() + 5,
                                              Math.toRadians(chambers.getHeading())),
                                   Math.toRadians(270));
    }
    public void init_toScoreSpeciment_1() {
        toScoreSpeciment_1 = drive.trajectorySequenceBuilder(current_pose)
            .strafeTo(chambers.vec());
    }
    public void init_toSubmersibleSide() {
        toSubmersibleSide = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(135)
            .splineToSplineHeading(submersibleSide, Math.toRadians(0));
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

        /* -----0----- 0*/

        init_toAlliance_0();
        drive.followTrajectorySequenceAsync(toAlliance_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        init_turn();
        drive.followTrajectorySequenceAsync(turn.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        /* -----0----- 1*/

        init_toAlliance_1();
        drive.followTrajectorySequenceAsync(toAlliance_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        init_turn();
        drive.followTrajectorySequenceAsync(turn.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        /* -----0----- 2*/

        init_toAlliance_2();
        drive.followTrajectorySequenceAsync(toAlliance_2.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        init_turn();
        drive.followTrajectorySequenceAsync(turn.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        /* -----0----- */

        init_toObservationZone_0();
        drive.followTrajectorySequenceAsync(toObservationZone_0.build());
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

        init_toObservationZone_1();
        drive.followTrajectorySequenceAsync(toObservationZone_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        /* --------- */

        init_toScoreSpeciment_0();
        drive.followTrajectorySequenceAsync(toScoreSpeciment_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //distance sensor here

        drive.setPoseEstimate(new Pose2d(
            calculateRealYLocation(current_pose, value), Math.toRadians(current_pose.getHeading())
        ));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        init_toScoreSpeciment_1();
        drive.followTrajectorySequenceAsync(toScoreSpeciment_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        /* -----1----- */
        chambers = new Pose2d(chambers.getX() + 2, chambers.getY(), Math.toRadians(chambers.getHeading()));

        init_toObservationZone_0();
        drive.followTrajectorySequenceAsync(toObservationZone_0.build());
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

        init_toObservationZone_1();
        drive.followTrajectorySequenceAsync(toObservationZone_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        /* --------- */

        init_toScoreSpeciment_0();
        drive.followTrajectorySequenceAsync(toScoreSpeciment_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //distance sensor here

        drive.setPoseEstimate(new Pose2d(
            calculateRealYLocation(current_pose, value), Math.toRadians(current_pose.getHeading())
        ));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        init_toScoreSpeciment_1();
        drive.followTrajectorySequenceAsync(toScoreSpeciment_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        /* -----2----- */
        chambers = new Pose2d(chambers.getX() + 2, chambers.getY(), Math.toRadians(chambers.getHeading()));

        init_toObservationZone_0();
        drive.followTrajectorySequenceAsync(toObservationZone_0.build());
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

        init_toObservationZone_1();
        drive.followTrajectorySequenceAsync(toObservationZone_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //mechanisms

        /* --------- */

        init_toScoreSpeciment_0();
        drive.followTrajectorySequenceAsync(toScoreSpeciment_0.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        //distance sensor here

        drive.setPoseEstimate(new Pose2d(
            calculateRealYLocation(current_pose, value), Math.toRadians(current_pose.getHeading())
        ));
        current_pose = fixedPose2d(drive.getPoseEstimate());

        init_toScoreSpeciment_1();
        drive.followTrajectorySequenceAsync(toScoreSpeciment_1.build());
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = fixedPose2d(drive.getPoseEstimate());

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
