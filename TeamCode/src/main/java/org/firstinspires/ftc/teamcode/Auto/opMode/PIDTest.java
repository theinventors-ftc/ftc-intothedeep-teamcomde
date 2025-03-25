package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
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
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;

@Config
@Autonomous(name = "PIDTest", group = "Tuning")
public class PIDTest extends OpMode {

    private Follower follower;
    private RobotMap robotMap;

    private Timer timer;
    private volatile Pose curr;

    private IntakeSubsystem in;

    public static int
        startHeading = 90,
        goalHeading = 90;

    private int
        pathState;

    private Pose
        startPose = new Pose(-2 * Tile, -Tile + robotY/2, Math.toRadians(90), false),

        start = new Pose(-2 * Tile, -Tile + robotY/2, Math.toRadians(startHeading), false),

        goal = new Pose(-2 * Tile, Tile + robotY/2, Math.toRadians(goalHeading), false);

    private Path
        starting;

    private PathChain
        forward,
        reverse;

    public void starting() {
        starting = new Path(new BezierLine(
            new Point(startPose),
            new Point(goal)
        ));
        starting.setLinearHeadingInterpolation(startPose.getHeading(), goal.getHeading());
        starting.setPathEndTimeoutConstraint(2000);

    }

    public void forward() {
        forward = follower.pathBuilder()
            .addPath(new BezierLine(
            new Point(goal),
            new Point(start)
        ))
        .setLinearHeadingInterpolation(goal.getHeading(), start.getHeading())
        .build();
    }

    public void reverse() {

    }

    public void setPathState(int pState) {
        pathState = pState;
        timer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(starting);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                }
                break;

//            case 1:
//                if (!follower.isBusy()) {
//                    reverse();
//                    follower.followPath(reverse);
//                    setPathState(2);
//                }
//                break;

//            case 2:
//                if (!follower.isBusy()) {
//                    forward();
//                    follower.followPath(forward);
//                    setPathState(1);
//                }
//                break;
        }
    }

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.AUTO);
        timer = new Timer();
        in = new IntakeSubsystem(robotMap);

        starting();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        curr = follower.getPose();
        autonomousPathUpdate();
        CommandScheduler.getInstance().run();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
        follower.drawOnDashBoard();
    }
}
