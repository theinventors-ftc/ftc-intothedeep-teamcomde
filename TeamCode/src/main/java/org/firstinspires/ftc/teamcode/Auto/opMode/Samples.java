package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;

@Autonomous(name = "Auto5Samples", group = "Special")
public class Samples extends OpMode {
    private Follower follower;
    private OpCommon opCommon;
    private RobotMap robotMap;
    private SequentialCommandGroup temp;
    private Timer timer;

    private RobotEx.Alliance alliance = RobotEx.Alliance.RED;

    private enum PathState {
        PRELOAD,
        RIGHT_SAMPLE,
        MID_SAMPLE,
        LEFT_SAMPLE,
        LEFT_SAMPLE_BAKSET,
        SUBMERSIBLE,
        SUBMERSIBLE_BASKET,
        SECOND_SUBMERSIBLE,
        SECOND_SUBMERSIBLE_BASKET,
        PARKING,
        STOP
    }

    private PathState pathState;

    private int
        xThreshold = 4,
        yThreshold = 4;

    private boolean
        curr = true;

    private Pose
        start = new Pose(-2 * Tile + robotX/2, -3 * Tile + robotY/2, Math.toRadians(90), false),

        sampleRight = new Pose(-59, -2 * Tile - 4, Math.toRadians(65), false),

        sampleMid = new Pose(-2.6 * Tile - 2, -1.93 * Tile, Math.toRadians(80), false),

        basket_3Sasmple = new Pose(-2.6 * Tile - 2, -1.93 * Tile + 3, Math.toRadians(80), false),

        sub_basket = new Pose(-2.55 * Tile, -2.1 * Tile, Math.toRadians(80), false),

        sec_sub_basket = new Pose(-2.55 * Tile, -2.1 * Tile, Math.toRadians(80), false),

        sampleLeft = new Pose(-2.45 * Tile, -1.48 * Tile, Math.toRadians(149), false),

        sub_side = new Pose(-1.25 * Tile, -0.5 * Tile, Math.toRadians(0), false),

        sec_sub_side = new Pose(-1.25 * Tile, -0.5 * Tile, Math.toRadians(0), false),

        parking = new Pose(-1.2 * Tile, -5, Math.toRadians(180), false),

        failSafe_parking = new Pose(-1.2 * Tile + 5, -5, Math.toRadians(180), false);

    private Path
        preload,
        park;

    private PathChain
        sample_2,
        sample_3,
        s3basket,
        submersible,
        subasket,
        sec_submersible,
        sec_subasket,
        failSafePark;

    public void buildPaths() {
        FollowerConstants.zeroPowerAccelerationMultiplier = 3.5;

        preload = new Path(new BezierLine(new Point(start), new Point(sampleRight)));
        preload.setLinearHeadingInterpolation(start.getHeading(), sampleRight.getHeading());

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

        sec_submersible = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(sub_basket),
                new Point(new Pose(-2.2 * Tile, -0.7 * Tile, false)),
                new Point(sec_sub_side)))
            .setLinearHeadingInterpolation(sub_basket.getHeading(), sec_sub_side.getHeading())
            .build();

        sec_subasket = follower.pathBuilder()
            .addPath(new BezierCurve(
                new Point(sec_sub_side),
                new Point(new Pose(-2 * Tile, 0, false)),
                new Point(sec_sub_basket)))
            .setLinearHeadingInterpolation(sec_sub_side.getHeading(), sec_sub_basket.getHeading())
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
                    new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                    new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(400)), // Prep Extendo
                    new ParallelCommandGroup(
                        opCommon.basket_scoring(),
                        new InstantCommand(() -> opCommon.armSubsystem.setWristState(
                            ArmSubsystem.WristState.BASKET_OUTTAKE
                        ))
                    )
                );
                temp.schedule();

                follower.followPath(preload, true);
                setPathState(PathState.RIGHT_SAMPLE);
                break;

            case RIGHT_SAMPLE:
                if (follower.atPose(sampleRight, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        opCommon.extendo(0.6),
                        opCommon.sample_intake()
                    );
                    temp.schedule();
                }

                if (!CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = new SequentialCommandGroup(
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(50))
                        , // Prep Extendo
                        opCommon.basket_scoring(),
                        new WaitCommand(200),
                        opCommon.release_sample(),
                        opCommon.extendo(0.35),
                        opCommon.sample_intake()
                    );
                    temp.schedule();

                    follower.followPath(sample_2, true);
                    setPathState(PathState.MID_SAMPLE);
                }
                break;

            case MID_SAMPLE:
                if (follower.atPose(sampleMid, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(250)),
                        // Prep Extendo
                        opCommon.basket_scoring(),
                        new WaitCommand(200),
                        opCommon.release_sample()
                    );
                    temp.schedule();
                }

                if (!CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = new SequentialCommandGroup(
                        new WaitCommand(1000),
                        opCommon.extendo3dSample(0.35),
                        opCommon.sample_intake()
                    );
                    temp.schedule();

                    follower.followPath(sample_3, true);
                    setPathState(PathState.LEFT_SAMPLE);
                }
                break;

            case LEFT_SAMPLE:
                if (!CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = new SequentialCommandGroup(
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.returnToZero()), // Prep Extendo
                        new ParallelCommandGroup(
                            opCommon.basket_scoring(),
                            new InstantCommand(() -> opCommon.armSubsystem.setWristState(
                                ArmSubsystem.WristState.BASKET_OUTTAKE
                            ))
                        )
                    );
                    temp.schedule();

                    follower.followPath(s3basket, true);
                    setPathState(PathState.LEFT_SAMPLE_BAKSET);
                }
                break;

            case LEFT_SAMPLE_BAKSET:
                if (follower.atPose(sampleMid, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.returnToZero()),
                        // Prep Extendo
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
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(250))
                        , // Prep Extendo
                        new InstantCommand(opCommon.intakeSubsystem::wiper_full_open),
                        new WaitCommand(400),
                        new InstantCommand(opCommon.intakeSubsystem::wiper_contract),
                        opCommon.extendo(0.35),
                        opCommon.sample_intake(),
                        new InstantCommand(opCommon.intakeSubsystem::wiper_contract)
                    );
                    temp.schedule();
                }

                if (!CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            opCommon.basket_scoring(),
                            new InstantCommand(() -> opCommon.armSubsystem.setWristState(
                                ArmSubsystem.WristState.BASKET_OUTTAKE
                            )),
                            new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                                new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(500)) // Prep Extendo
                            ))
                    );
                    temp.schedule();

                    follower.followPath(subasket, true);
                    setPathState(PathState.SUBMERSIBLE_BASKET);
                }
                break;

            case SUBMERSIBLE_BASKET:
                if (follower.atPose(sub_basket, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.returnToZero()),
                        // Prep Extendo
                        opCommon.reset_elevator()
                    );
                    temp.schedule();

                    follower.followPath(sec_submersible, true);
                    setPathState(PathState.SECOND_SUBMERSIBLE);
                }
                break;

            case SECOND_SUBMERSIBLE:
                if (follower.atPose(sec_sub_side, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(250))
                        , // Prep Extendo
                        opCommon.extendo(0.35),
                        opCommon.sample_intake(),
                        new InstantCommand(opCommon.intakeSubsystem::wiper_contract)
                    );
                    temp.schedule();
                }

                if (!CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            opCommon.basket_scoring(),
                            new InstantCommand(() -> opCommon.armSubsystem.setWristState(
                                ArmSubsystem.WristState.BASKET_OUTTAKE
                            )),
                            new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                                new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(500)) // Prep Extendo
                            ))
                    );
                    temp.schedule();

                    follower.followPath(subasket, true);
                    setPathState(PathState.SECOND_SUBMERSIBLE_BASKET);
                }
                break;

            case SECOND_SUBMERSIBLE_BASKET:
                if (follower.atPose(sec_sub_basket, xThreshold, yThreshold) && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample()
                    );
                    temp.schedule();
                }

                if (!CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = new SequentialCommandGroup(
                        opCommon.reset_elevator(),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(-50)),
                        new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(-50)),
                        opCommon.parking()
                    );
                    temp.schedule();

                    follower.followPath(park, true);
                    setPathState(PathState.PARKING);
                }
                break;

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

        if (pathState == PathState.SUBMERSIBLE && timer.getElapsedTime() >= 6000) {
            setPathState(PathState.SECOND_SUBMERSIBLE);
            telemetry.addData("FailSafe: ", 1);
            curr = true;
        }
    }
}
