package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.constants.FConstants;
import org.firstinspires.ftc.teamcode.Auto.constants.LConstants;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;

@Autonomous(name = "Specimen al Americana", group = "Special")
public class Specimen extends OpMode {
    private Follower follower;
    private OpCommon opCommon;
    private RobotMap robotMap;
    private SequentialCommandGroup temp;
    private Timer timer;

    private RobotEx.Alliance alliance = RobotEx.Alliance.RED;

    private double
        xThreshold = 4,
        yThreshold = 4,
        xThreshold_obs = 2.5,
        yThreshold_obs = 2.5,
        hThreshold = 0.05,
        failSafetime = 6000;

    private boolean
        curr = true;

    private enum PathState {
        PRELOAD,
        PRELOAD_RELEASE,
        LEFT_SAMPLE,
        MID_SAMPLE,
        OBSERVATION_CHAMBER_0,
        CHAMBER_OBSERVATION_0,
        OBSERVATION_CHAMBER_1,
        CHAMBER_OBSERVATION_1,
        OBSERVATION_CHAMBER_2,
        CHAMBER_OBSERVATION_2,
        OBSERVATION_CHAMBER_3,
        CHAMBER_OBSERVATION_3,
        PARKING,
        STOP
    }

    private PathState pathState;

    private Pose
        start = new Pose(
            Tile - robotX/2, (-3 * Tile) + robotY/2 + 1, Math.toRadians(270), false),

        preload = new Pose(
            -9 , -Tile - (robotY/2) + 1, Math.toRadians(270), false),

        chambers_0 = new Pose(
            -4 , -Tile - (robotY/2) + 1, Math.toRadians(270), false),

        chambers_1 = new Pose(chambers_0.getX() + 3, chambers_0.getY(), chambers_0.getHeading(),
                              false),

        chambers_2 = new Pose(chambers_0.getX() + 6, chambers_0.getY(), chambers_0.getHeading(),
                              false),

        chambers_3 = new Pose(chambers_0.getX() + 9, chambers_0.getY(), chambers_0.getHeading(),
                              false),

        observationZone_first = new Pose(
            3 * Tile - (robotX/2), -3 * Tile + (robotY/2) - 1,
            Math.toRadians(270), false),

        observationZone = new Pose(
            1.5 * Tile, -3 * Tile + (robotY/2),
            Math.toRadians(280), false),

        allianceSampleLeft = new Pose(
            2 * Tile - 1, -Tile + (robotY/2), Math.toRadians(270), false),

        allianceSampleMid = new Pose(
            2.5 * Tile - 4, -Tile + (robotY/2), Math.toRadians(270), false),

        allianceSampleRight = new Pose(
            3 * Tile - (robotX/2) - 2, -Tile + (robotY/2), Math.toRadians(270), false),

        parking = new Pose(
            2 * Tile, -2.4 * Tile, Math.toRadians(315), false);

    private Path
        toPreload,
        toParking;

    private PathChain
        samples,
        toObservationZone_0,
        toObservationZone_1,
        toObservationZone_2,
        toObservationZone_3,
        toScoreSpeciment_0,
        toScoreSpeciment_1,
        toScoreSpeciment_2,
        toScoreSpeciment_3;

    public void buildPaths() {
        toPreload = new Path(new BezierLine(new Point(start), new Point(preload)));
        toPreload.setConstantHeadingInterpolation(preload.getHeading());

        //*//

        samples = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(preload),
                new Point(new Pose(2.3 * Tile, -2.3 * Tile, false)),
                new Point(new Pose(1.3 * Tile, 8, false)),
                new Point(allianceSampleLeft)
            ))
            .setConstantHeadingInterpolation(Math.toRadians(270))
            .addPath(new BezierLine(
                new Point(allianceSampleLeft),
                new Point(new Pose(allianceSampleLeft.getX(), -2.5 * Tile + 22.4, false))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(270))
            .addPath(new BezierCurve(
                new Point(new Pose(allianceSampleLeft.getX(), -2.5 * Tile + 22.4, false)),
                new Point(allianceSampleLeft),
                new Point(new Pose(2.25 * Tile, 3, false)),
                new Point(allianceSampleMid)
            ))
            .setConstantHeadingInterpolation(Math.toRadians(270))
            .addPath(new BezierLine(
                new Point(allianceSampleMid),
                new Point(new Pose(allianceSampleMid.getX(), -2.5 * Tile + 22.4, false))
            ))
            .setConstantHeadingInterpolation(Math.toRadians(270))
            .addPath(new BezierCurve(
                new Point(new Pose(allianceSampleMid.getX(), -2.5 * Tile + 22.4, false)),
                new Point(allianceSampleMid),
                new Point(new Pose(2.55 * Tile, 3, false)),
                new Point(allianceSampleRight)
            ))
            .setConstantHeadingInterpolation(Math.toRadians(270))
            .addPath(new BezierLine(
                new Point(allianceSampleRight),
                new Point(observationZone_first)
            ))
            .setConstantHeadingInterpolation(Math.toRadians(270))
            .build();

        //*//

        toScoreSpeciment_0 = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(observationZone_first),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(chambers_0)
            ))
            .setLinearHeadingInterpolation(observationZone_first.getHeading(),
                                           chambers_0.getHeading())
            .build();

        toScoreSpeciment_1 = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(observationZone),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(chambers_1)
            ))
            .setLinearHeadingInterpolation(observationZone.getHeading(), chambers_1.getHeading())
            .build();

        toScoreSpeciment_2 = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(observationZone),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(chambers_2)
            ))
            .setLinearHeadingInterpolation(observationZone.getHeading(), chambers_2.getHeading())
            .build();

        toScoreSpeciment_3 = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(observationZone),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(chambers_3)
            ))
            .setLinearHeadingInterpolation(observationZone.getHeading(), chambers_3.getHeading())
            .build();

        toObservationZone_0 = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(chambers_0),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(new Pose(1.5 * Tile, -2 * Tile, false)),
                new Point(observationZone)
            ))
            .setLinearHeadingInterpolation(chambers_0.getHeading(), observationZone.getHeading())
            .build();

        toObservationZone_1 = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(chambers_1),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(new Pose(1.5 * Tile, -2 * Tile, false)),
                new Point(observationZone)
            ))
            .setLinearHeadingInterpolation(chambers_1.getHeading(), observationZone.getHeading())
            .build();

        toObservationZone_2 = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(chambers_2),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(new Pose(1.5 * Tile, -2 * Tile, false)),
                new Point(observationZone)
            ))
            .setLinearHeadingInterpolation(chambers_2.getHeading(), observationZone.getHeading())
            .build();

        toObservationZone_3 = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(chambers_3),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(new Pose(1.5 * Tile, -2 * Tile, false)),
                new Point(observationZone)
            ))
            .setLinearHeadingInterpolation(chambers_3.getHeading(), observationZone.getHeading())
            .build();

        toParking = new Path(new BezierLine(
            new Point(chambers_3),
            new Point(observationZone)));
        toParking.setLinearHeadingInterpolation(chambers_3.getHeading(), observationZone.getHeading());
    }

    public void setPathState(PathState pState) {
        pathState = pState;
        timer.resetTimer();
    }

    public boolean timeFailSafeCheck() {
        return timer.getElapsedTime() >= failSafetime;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD:
                temp = opCommon.scoreSpeciment();
                temp.schedule();

                follower.followPath(toPreload, true);
                setPathState(PathState.PRELOAD_RELEASE);
                break;

            case PRELOAD_RELEASE:
                if (follower.atPose(preload, xThreshold, yThreshold) && curr) {
                    curr = false;
                    temp = opCommon.releaseSpecimen();
                    temp.schedule();
                }

                if (!curr) {
                    curr = true;
                    temp = new SequentialCommandGroup(
                        opCommon.specimenAim(),
                        new InstantCommand(opCommon.clawSubsystem::looslyGripped),
                        new InstantCommand(opCommon.intakeSubsystem::lower)
                    );
                    temp.schedule();

                    follower.followPath(samples, true);
                    setPathState(PathState.LEFT_SAMPLE);
                }
                break;

            case LEFT_SAMPLE:
                if (follower.atPose(new Pose(1.5 * Tile, -Tile), 10, 10)) {
                    follower.setMaxPower(0.8);
                }

                if (follower.getCurrentTValue() >= 0.8 &&
                    follower.atPose(allianceSampleLeft, 6, 6, hThreshold) && !CommandScheduler.getInstance().isScheduled(temp)) {

                    follower.setMaxPower(1);

                    temp = opCommon.extendoSpecimenPush();
                    temp.schedule();

                    setPathState(PathState.MID_SAMPLE);
                }
                break;

            case MID_SAMPLE:
                if (follower.atPose(allianceSampleLeft, 6, 6)) {
                    follower.setMaxPower(0.9);
                }

                if (follower.atPose(allianceSampleMid, xThreshold, yThreshold, hThreshold) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    follower.setMaxPower(1);
                    curr = true;
                    temp = new SequentialCommandGroup(
                        opCommon.extendoSpecimenPush(),
                        new InstantCommand(opCommon.intakeSubsystem::raise)
                    );
                    temp.schedule();

                    setPathState(PathState.OBSERVATION_CHAMBER_0);
                }
                break;

            case OBSERVATION_CHAMBER_0:
                if (!CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        new InstantCommand(opCommon.clawSubsystem::release)
                    );
                    temp.schedule();
                }

                if (follower.atPose(observationZone_first, 3, 3) && !CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = opCommon.scoreSpeciment();
                    temp.schedule();

                    follower.followPath(toScoreSpeciment_0, true);
                    setPathState(PathState.CHAMBER_OBSERVATION_0);
                }
                break;

            case CHAMBER_OBSERVATION_0:
                if (timeFailSafeCheck() || follower.atPose(chambers_0, xThreshold, yThreshold) && curr) {
                    curr = false;
                    temp = opCommon.releaseSpecimen();
                    temp.schedule();
                }

                if (follower.atPose(chambers_0, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = opCommon.specimenAim();
                    temp.schedule();

                    follower.followPath(toObservationZone_0, true);
                    setPathState(PathState.OBSERVATION_CHAMBER_1);
                }
                break;

            //*// 2

            case OBSERVATION_CHAMBER_1:
                if (follower.atPose(observationZone, xThreshold_obs, yThreshold_obs) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = opCommon.scoreSpeciment();
                    temp.schedule();

                    follower.followPath(toScoreSpeciment_1, true);
                    setPathState(PathState.CHAMBER_OBSERVATION_1);
                }
                break;

            case CHAMBER_OBSERVATION_1:
                if (timeFailSafeCheck() || follower.atPose(chambers_1, xThreshold, yThreshold) && curr) {
                    curr = false;
                    temp = opCommon.releaseSpecimen();
                    temp.schedule();
                }

                if (follower.atPose(chambers_1, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = opCommon.specimenAim();
                    temp.schedule();

                    follower.followPath(toObservationZone_1, true);
                    setPathState(PathState.OBSERVATION_CHAMBER_2);
                }
                break;

            //*// 3

            case OBSERVATION_CHAMBER_2:
                if (follower.atPose(observationZone, xThreshold_obs, yThreshold_obs) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = opCommon.scoreSpeciment();
                    temp.schedule();

                    follower.followPath(toScoreSpeciment_2, true);
                    setPathState(PathState.CHAMBER_OBSERVATION_2);
                }
                break;

            case CHAMBER_OBSERVATION_2:
                if (timeFailSafeCheck() || follower.atPose(chambers_2, xThreshold, yThreshold) && curr) {
                    curr = false;
                    temp = opCommon.releaseSpecimen();
                    temp.schedule();
                }

                if (follower.atPose(chambers_2, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = opCommon.specimenAim();
                    temp.schedule();

                    follower.followPath(toObservationZone_2, true);
                    setPathState(PathState.OBSERVATION_CHAMBER_3);
                }
                break;

            //*// 4

            case OBSERVATION_CHAMBER_3:
                if (follower.atPose(observationZone, xThreshold_obs, yThreshold_obs) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = opCommon.scoreSpeciment();
                    temp.schedule();

                    follower.followPath(toScoreSpeciment_3, true);
                    setPathState(PathState.CHAMBER_OBSERVATION_3);
                }
                break;

            case CHAMBER_OBSERVATION_3:
                if (timeFailSafeCheck() || follower.atPose(chambers_3, xThreshold, yThreshold) && curr) {
                    curr = false;
                    temp = opCommon.releaseSpecimen();
                    temp.schedule();
                }

                if (follower.atPose(chambers_3, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = opCommon.armReset();
                    temp.schedule();

                    follower.followPath(toObservationZone_3, true);
                    setPathState(PathState.PARKING);
                }
                break;

            //*// 5

            case PARKING:
                if(follower.atPose(parking, xThreshold, yThreshold)) {
                    setPathState(PathState.STOP);
                }
                break;
        }
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.AUTO);
        follower = new Follower(robotMap, FConstants.class, LConstants.class);
        follower.setStartingPose(start);
        opCommon = new OpCommon(robotMap, alliance);
        timer = new Timer();

        buildPaths();
    }

    @Override
    public void start() {
        setPathState(PathState.PRELOAD);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        CommandScheduler.getInstance().run();
        follower.drawOnDashBoard();

        telemetry.addData("Pathstate", pathState);
        telemetry.addData("Is Robot Stuck", follower.isRobotStuck());
    }

    @Override
    public void stop() {
        PoseStorage.currentPose = new Pose2d(
                follower.getPose().getAsFTCStandardCoordinates().getX(),
                follower.getPose().getAsFTCStandardCoordinates().getY(),
                follower.getPose().getAsFTCStandardCoordinates().getHeading()
        );
    }
}