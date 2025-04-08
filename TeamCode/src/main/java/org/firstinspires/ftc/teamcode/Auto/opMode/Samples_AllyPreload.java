package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
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
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;

@Autonomous(name = "Auto6Samples", group = "Special")
public class Samples_AllyPreload extends OpMode {
    private Follower follower;
    private OpCommon opCommon;
    private RobotMap robotMap;
    private SequentialCommandGroup temp;
    private Timer timer;

    private RobotEx.Alliance alliance = RobotEx.Alliance.RED;

    private double
        xThreshold = 4,
        yThreshold = 4,
        failSafetime = 10000;

    private boolean
        curr = true;

    private enum PathState {
        PRELOAD,
        ALLY_PRELOAD,
        RIGHT_SAMPLE,
        MID_SAMPLE,
        LEFT_SAMPLE,
        LEFT_SAMPLE_BAKSET,
        SUBMERSIBLE,
        SUBMERSIBLE_BASKET,
        PARKING,
        STOP
    }

    private PathState pathState;

    private Pose
        start = new Pose(-2 * Tile + robotY/2, -3 * Tile + robotX/2, Math.toRadians(0), false),

        allyPreload = new Pose(-Tile, -3 * Tile + robotX/2, Math.toRadians(0), false),

        preload_basket = new Pose(-2 * Tile + robotY/2 - 3, -3 * Tile + robotX/2, Math.toRadians(0),
                                  false),

        sampleRight = new Pose(-59, -2 * Tile - 4, Math.toRadians(65), false),

        sampleMid = new Pose(-2.6 * Tile - 2, -1.93 * Tile, Math.toRadians(80), false),

        basket_3Sasmple = new Pose(-2.6 * Tile - 2, -1.93 * Tile + 3, Math.toRadians(80), false),

        sub_basket = new Pose(-2.65 * Tile, -1.85 * Tile, Math.toRadians(80), false),

        sampleLeft = new Pose(-2.45 * Tile, -1.48 * Tile, Math.toRadians(153), false),

        sub_side = new Pose(-1.3 * Tile, -0.5 * Tile, Math.toRadians(0), false),

        parking = new Pose(-1.2 * Tile, -5, Math.toRadians(180), false),

        failSafe_parking = new Pose(-1.2 * Tile + 5, -5, Math.toRadians(180), false);

    private Path
        preload,
        park;

    private PathChain
        sample_allyPreload,
        allyPreload_score,
        sample_2,
        sample_3,
        s3basket,
        submersible,
        subasket,
        failSafePark;

    public void buildPaths() {
        preload = new Path(new BezierLine(new Point(start), new Point(preload_basket)));
        preload.setConstantHeadingInterpolation(preload_basket.getHeading());

        sample_allyPreload = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(preload_basket), new Point(allyPreload)))
            .setConstantHeadingInterpolation(preload_basket.getHeading())
            .build();

        allyPreload_score = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(allyPreload), new Point(sampleRight)))
            .setLinearHeadingInterpolation(allyPreload.getHeading(), sampleRight.getHeading())
            .build();

        sample_2 = follower.pathBuilder()
            .addPath(new BezierLine(
            new Point(sampleRight), new Point(sampleMid)))
            .setLinearHeadingInterpolation(sampleRight.getHeading(), sampleMid.getHeading())
            .build();

        sample_3 = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(sampleMid), new Point(sampleLeft)))
            .setLinearHeadingInterpolation(sampleMid.getHeading(), sampleLeft.getHeading())
            .build();

        s3basket = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(sampleLeft), new Point(basket_3Sasmple)))
            .setLinearHeadingInterpolation(sampleLeft.getHeading(), basket_3Sasmple.getHeading())
            .build();

        submersible = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(basket_3Sasmple),
                new Point(new Pose(-2.2 * Tile, -0.7 * Tile, false)),
                new Point(sub_side)))
            .setLinearHeadingInterpolation(basket_3Sasmple.getHeading(), sub_side.getHeading())
            .build();

        subasket = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(sub_side),
                new Point(new Pose(-2 * Tile, 0, false)),
                new Point(sub_basket)))
            .setLinearHeadingInterpolation(sub_side.getHeading(), sub_basket.getHeading())
            .build();

        park = new Path(new BezierCurve(
            new Point(basket_3Sasmple),
            new Point(new Pose(-2.5 * Tile, 0, false)),
            new Point(parking)));
        park.setLinearHeadingInterpolation(basket_3Sasmple.getHeading(), parking.getHeading());

        //----Fail Safe Programs----//

        failSafePark = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(sub_side),
                new Point(new Pose(-2 * Tile, failSafe_parking.getY(), false))
            ))
            .setLinearHeadingInterpolation(sub_side.getHeading(), failSafe_parking.getHeading())
            .addPath(new BezierLine(
                new Point(new Pose(-2 * Tile, failSafe_parking.getY(), false)),
                new Point(failSafe_parking)
            ))
            .setConstantHeadingInterpolation(failSafe_parking.getHeading())
            .build();
    }

    public void setPathState(PathState pState) {
        pathState = pState;
        timer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case PRELOAD:
                temp = new SequentialCommandGroup(
                    opCommon.basket_scoring(),
                    new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                    new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(400)) // Prep Extendo
                );
                temp.schedule();

                follower.followPath(preload, true);
                setPathState(PathState.ALLY_PRELOAD);
                break;

            case ALLY_PRELOAD:
                if (follower.atPose(preload_basket, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        opCommon.extendo(0.6),
                        opCommon.sample_intake()
                    );
                    temp.schedule();

                    follower.followPath(sample_allyPreload, true);
                    setPathState(PathState.RIGHT_SAMPLE);
                }
                break;

            case RIGHT_SAMPLE:
                if (timer.getElapsedTime() >= failSafetime ||
                    !CommandScheduler.getInstance().isScheduled(temp))
                {
                    temp = new SequentialCommandGroup(
                        opCommon.basket_scoring(),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(400)) // Prep Extendo
                    );
                    temp.schedule();

                    follower.followPath(allyPreload_score, true);
                    setPathState(PathState.MID_SAMPLE);
                }
                break;

            case MID_SAMPLE:
                if (follower.atPose(sampleRight, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = new SequentialCommandGroup(
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(400)),
                        opCommon.release_sample(),
                        // Prep Extendo
                        opCommon.extendo(0.6),
                        opCommon.sample_intake(),
                        new WaitUntilCommand(() -> opCommon.sample_intake().isFinished()),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(50)),
                        opCommon.basket_scoring(),
                        opCommon.release_sample(),
                        opCommon.extendo(0.35),
                        opCommon.sample_intake()
                    );
                    temp.schedule();

                    follower.followPath(sample_2, true);
                    setPathState(PathState.LEFT_SAMPLE);
                }
                break;

            case LEFT_SAMPLE:
                if (follower.atPose(sampleMid, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(250))
                        , // Prep Extendo
                        new WaitCommand(1000),
                        opCommon.extendo3dSample(0.35),
                        opCommon.sample_intake()
                    );
                    temp.schedule();

                    follower.followPath(sample_3, true);
                    setPathState(PathState.LEFT_SAMPLE_BAKSET);
                }
                break;

            case LEFT_SAMPLE_BAKSET:
                if (!CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = opCommon.basket_scoring();
                    temp.schedule();

                    follower.followPath(s3basket, true);
                }

                if (follower.atPose(sampleMid, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        opCommon.reset_elevator()
                    );
                    temp.schedule();

                    follower.followPath(submersible, true);
                    setPathState(PathState.SUBMERSIBLE);
                }
                break;

            case SUBMERSIBLE:
                if (follower.atPose(sub_side, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(600)), // Prep Extendo
                        opCommon.extendo(0.6),
                        opCommon.sample_intake(),
                        new InstantCommand(() -> opCommon.intakeSubsystem.reverse()),
                        new WaitCommand(150),
                        new InstantCommand(() -> opCommon.intakeSubsystem.stop())
                    );
                    temp.schedule();
                }

                if (!CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    setPathState(PathState.SUBMERSIBLE_BASKET);
                }
                break;

            case SUBMERSIBLE_BASKET:
                if (!CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = opCommon.basket_scoring();
                    temp.schedule();
                    follower.followPath(subasket, true);
                }

                if (follower.atPose(sampleMid, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        opCommon.reset_elevator()
                    );
                    temp.schedule();

                    setPathState(PathState.PARKING);
                }

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
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(start);
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.AUTO);
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
    }
}
