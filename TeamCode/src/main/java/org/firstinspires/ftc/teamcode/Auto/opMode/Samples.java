package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;

import com.arcrobotics.ftclib.command.Command;
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
import com.pedropathing.util.Constants;
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

    private int
        pathState;

    private boolean
        curr = true;

    private Pose
        start = new Pose(-2 * Tile + robotX/2, -3 * Tile + robotY/2, Math.toRadians(90), false),

        sampleRight = new Pose(-57, -2 * Tile - 2, Math.toRadians(67), false),

        sampleMid = new Pose(-2.6 * Tile, -1.93 * Tile, Math.toRadians(80), false),

        sampleLeft = new Pose(-2.45 * Tile, -1.6 * Tile, Math.toRadians(140), false),

        sub_side = new Pose(-1.3 * Tile, -0.5 * Tile, Math.toRadians(0), false),

        parking = new Pose(-1.2 * Tile, -5, Math.toRadians(180), false);

    private Path
        preload,
        park;

    private PathChain
        sample_2,
        sample_3,
        s3basket,
        submersible,
        subasket,
        failSafePark;

    public void buildPaths() {
        preload = new Path(new BezierLine(
            new Point(start),
            new Point(sampleRight)
        ));
        preload.setLinearHeadingInterpolation(start.getHeading(), sampleRight.getHeading());

        sample_2 = follower.pathBuilder()
            .addPath(new BezierLine(
            new Point(sampleRight),
            new Point(sampleMid)
            ))
            .setLinearHeadingInterpolation(sampleRight.getHeading(), sampleMid.getHeading())
            .build();

        sample_3 = follower.pathBuilder()
            .addPath(new BezierLine(
            new Point(sampleMid),
            new Point(sampleLeft)
            ))
            .setLinearHeadingInterpolation(sampleMid.getHeading(), sampleLeft.getHeading())
            .build();

        s3basket = follower.pathBuilder()
            .addPath(new BezierLine(
            new Point(sampleLeft),
            new Point(sampleMid)
            ))
            .setLinearHeadingInterpolation(sampleLeft.getHeading(), sampleMid.getHeading())
            .build();

        submersible = follower.pathBuilder()
            .addPath(new BezierCurve(
            new Point(sampleRight),
            new Point(new Pose(-2 * Tile, 0, false)),
            new Point(sub_side)
            ))
            .setLinearHeadingInterpolation(sampleRight.getHeading(), sub_side.getHeading())
            .build();

        subasket = follower.pathBuilder()
            .addPath(new BezierCurve(
            new Point(sub_side),
            new Point(new Pose(-2 * Tile, 0, false)),
            new Point(sampleMid)
            ))
            .setLinearHeadingInterpolation(sub_side.getHeading(), sampleMid.getHeading())
            .build();

        park = new Path(new BezierCurve(
            new Point(sampleMid),
            new Point(new Pose(-2.5 * Tile, 10, false)),
            new Point(parking)
        ));
        park.setLinearHeadingInterpolation(sampleMid.getHeading(), parking.getHeading());

        //----Fail Safe Programs----//

        failSafePark = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(sub_side),
                new Point(new Pose(-2 * Tile, parking.getY(), false))
            ))
            .setLinearHeadingInterpolation(sub_side.getHeading(), parking.getHeading())
            .addPath(new BezierLine(
                new Point(new Pose(-2 * Tile, parking.getY(), false)),
                new Point(parking)
            ))
            .setConstantHeadingInterpolation(parking.getHeading())
            .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        timer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                temp = new SequentialCommandGroup(
                    new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                    new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(400)), // Prep Extendo
                    opCommon.basket_scoring()
                );

                temp.schedule();
                follower.followPath(preload);
                setPathState(1);
                break;

            case 1:
                if (follower.atPose(sampleRight, 1, 1) && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
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
                        new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(150)),
                        new WaitCommand(200),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(50))
                        , // Prep Extendo
                        opCommon.basket_scoring(),
                        new WaitCommand(50),
                        opCommon.release_sample(),
                        opCommon.extendo(0.35),
                        opCommon.sample_intake()
                    );
                    temp.schedule();

                    follower.followPath(sample_2);
                    setPathState(2);
                }
                break;

            case 2:
                if (follower.atPose(sampleMid, 1, 1) && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(150)),
                        new WaitCommand(200),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(250)), // Prep Extendo
                        opCommon.basket_scoring(),
                        new WaitCommand(50),
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

                    follower.followPath(sample_3);
                    setPathState(3);
                }
                break;

            case 3:
                if (follower.atPose(sampleLeft, 1, 1) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = new SequentialCommandGroup(
                        new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(150)),
                        new WaitCommand(200),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.returnToZero()), // Prep Extendo
                        new WaitCommand(400),
                        opCommon.basket_scoring()
                    );
                    temp.schedule();

                    sampleRight.setX(sampleRight.getX() - 2);
                    sampleRight.setY(sampleRight.getY() + 2);

                    follower.followPath(s3basket);
                    setPathState(4);
                }
                break;

            case 4:
                if (follower.atPose(sampleMid, 1, 1) && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(500)), // Prep Extendo
                        opCommon.reset_elevator()
                    );
                    temp.schedule();

                    FollowerConstants.xMovement = 80;

                    follower.followPath(submersible, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (follower.atPose(sub_side, 1, 1) && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(600)), // Prep Extendo
                        opCommon.deep_intake(),
                        opCommon.submersible_intake(),
                        new InstantCommand(() -> opCommon.intakeSubsystem.reverse()),
                        new WaitCommand(150),
                        new InstantCommand(() -> opCommon.intakeSubsystem.stop())
                    );
                    temp.schedule();
                }

                if (!CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            opCommon.basket_scoring(),
                            new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                                new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(500)) // Prep Extendo
                            ))
                    );
                    temp.schedule();

                    sampleMid.setY(-1.85 * Tile);
                    sampleMid.setX(-2.65 * Tile);

                    follower.followPath(subasket);
                    setPathState(6);
                }
                break;

            case 6:
                if (follower.atPose(sampleMid, 1, 1) && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        new WaitCommand(200)
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

                    FollowerConstants.xMovement = 75;

                    follower.followPath(park);
                    setPathState(8);
                }
                break;

            case 7:
                if (curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        new InstantCommand(opCommon.intakeSubsystem::raise),
                        new InstantCommand(opCommon.intakeSubsystem::stop),
                        new InstantCommand(opCommon.intakeSubsystem::reverse),
                        new InstantCommand(() -> opCommon.armSubsystem.setArmState(
                            ArmSubsystem.ArmState.PARK
                        )),
                        new InstantCommand(() -> opCommon.armSubsystem.setWristState(
                            ArmSubsystem.WristState.PARK
                        )),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(-50)),
                        new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(-50)),
                        new WaitUntilCommand(() -> opCommon.elevatorSubsystem.atTarget()),
                        new WaitUntilCommand(() -> opCommon.extendoSubsystem.atTarget()),
                        new InstantCommand(opCommon.intakeSubsystem::stop)
                    );
                    temp.schedule();
                }

                if (!curr) {
                    temp =  opCommon.parking();
                    temp.schedule();

                    follower.followPath(failSafePark);
                    setPathState(8);
                }
                break;

            case 8:
                if(follower.atPose(parking, 1, 1)) {
                    setPathState(-1);
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
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        CommandScheduler.getInstance().run();

        if (pathState == 5 && timer.getElapsedTime() >= 6000) {
            setPathState(7);
            telemetry.addData("FailSafe: ", 1);
        }
    }
}
