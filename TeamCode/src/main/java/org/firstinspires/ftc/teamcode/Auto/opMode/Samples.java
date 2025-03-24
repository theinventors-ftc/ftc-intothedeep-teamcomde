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
    private volatile Pose currP;
    private OpCommon opCommon;
    private RobotMap robotMap;
    private SequentialCommandGroup temp;
    private Timer timer;

    private RobotEx.Alliance alliance = RobotEx.Alliance.RED;

    private int
        pathState;

    private boolean curr = true;

    private Pose
        start = new Pose(-2 * Tile + robotX/2, -3 * Tile + robotY/2, Math.toRadians(90), false),

        sampleRight = new Pose(-57, -2 * Tile - 2, Math.toRadians(67), false),

        sampleMid = new Pose(-2.6 * Tile, -1.93 * Tile, Math.toRadians(80), false),

        sampleLeft = new Pose(-2.35 * Tile, -1.46 * Tile, Math.toRadians(147), false),

        sub_side = new Pose(-1.3 * Tile, -0.5 * Tile, Math.toRadians(0), false),

        parking = new Pose(-1.2 * Tile, -5, Math.toRadians(180), false);

    private Path
        preload,
        sample_2,
        sample_3,
        s3basket,
        submersible,
        subasket,
        park;

    private PathChain failSafePark;

    public void preload() {
        preload = new Path(new BezierLine(
            new Point(start),
            new Point(sampleRight)
        ));
        preload.setLinearHeadingInterpolation(start.getHeading(), sampleRight.getHeading());
    }

    public void sample_2() {
        sample_2 = new Path(new BezierLine(
            new Point(currP),
            new Point(sampleMid)
        ));
        sample_2.setLinearHeadingInterpolation(currP.getHeading(), sampleMid.getHeading());
    }

    public void sample_3() {
        sample_3 = new Path(new BezierLine(
            new Point(currP),
            new Point(sampleLeft)
        ));
        sample_3.setLinearHeadingInterpolation(currP.getHeading(), sampleLeft.getHeading());
    }

    public void s3basket() {
        s3basket = new Path(new BezierLine(
            new Point(currP),
            new Point(sampleRight)
        ));
        s3basket.setLinearHeadingInterpolation(currP.getHeading(), sampleRight.getHeading());
    }

    public void submersible() {
        submersible = new Path(new BezierCurve(
            new Point(currP),
            new Point(new Pose(-2 * Tile, 0, false)),
            new Point(sub_side)
        ));
        submersible.setLinearHeadingInterpolation(currP.getHeading(), sub_side.getHeading());
    }

    public void subasket() {
        subasket = new Path(new BezierCurve(
            new Point(currP),
            new Point(new Pose(-2 * Tile, 0, false)),
            new Point(sampleMid)
        ));
        subasket.setLinearHeadingInterpolation(currP.getHeading(), sampleMid.getHeading());
    }

    public void park() {
        park = new Path(new BezierCurve(
            new Point(currP),
            new Point(new Pose(-2.5 * Tile, 10, false)),
            new Point(parking)
        ));
        park.setLinearHeadingInterpolation(currP.getHeading(), parking.getHeading());
    }

    public void failSafePark() {
        failSafePark = follower.pathBuilder()
            .addPath(new BezierLine(
                new Point(currP),
                new Point(new Pose(-2 * Tile, parking.getY(), false))
            ))
            .setLinearHeadingInterpolation(currP.getHeading(), parking.getHeading())
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
                preload();

                temp.schedule();
                follower.followPath(preload);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy() && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        opCommon.extendo(0.6),
                        opCommon.sample_intake(),
                        new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(150)),
                        new WaitCommand(200),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(50))
                        , // Prep Extendo
                        opCommon.basket_scoring(),
                        new WaitCommand(50),
                        opCommon.release_sample()
                    );
                    temp.schedule();
                }

                if (!CommandScheduler.getInstance().isScheduled(temp) && !curr) {
                    curr = true;
                    temp = new SequentialCommandGroup(
                        opCommon.extendo(0.35),
                        opCommon.sample_intake()
                    );
                    temp.schedule();

                    sample_2();
                    follower.followPath(sample_2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
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

                    sample_3();
                    follower.followPath(sample_3);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy() && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = new SequentialCommandGroup(
                        new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(150)),
                        new WaitCommand(200),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(600)), // Prep Extendo
                        new WaitCommand(1500),
                        opCommon.basket_scoring()
                    );
                    temp.schedule();

                    sampleRight.setX(sampleRight.getX() - 2);
                    sampleRight.setY(sampleRight.getY() + 2);

                    s3basket();
                    follower.followPath(s3basket);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy() && !CommandScheduler.getInstance().isScheduled(temp)) {
                    temp = new SequentialCommandGroup(
                        opCommon.release_sample(),
                        new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                        new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(500)), // Prep Extendo
                        opCommon.reset_elevator()
                    );
                    temp.schedule();

                    submersible();
                    follower.followPath(submersible);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy() && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
                    curr = false;
                    temp = new SequentialCommandGroup(
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

                    subasket();
                    follower.followPath(subasket);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy() && !CommandScheduler.getInstance().isScheduled(temp) && curr) {
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

                    park();
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

                    failSafePark();
                    follower.followPath(failSafePark);
                    setPathState(8);
                }
                break;

            case 8:
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(start);
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.AUTO);
        opCommon = new OpCommon(robotMap, alliance);
        timer = new Timer();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        currP = follower.getPose();
        CommandScheduler.getInstance().run();

        if (pathState == 5 && timer.getElapsedTime() >= 6000) {
            setPathState(7);
            telemetry.addData("FailSafe: ", 1);
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
        follower.drawOnDashBoard();
    }
}
