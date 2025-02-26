package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.opMode.OpCommon;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;

@Autonomous(name = "Auto Dummy", group = "Tests")
public class AutoDummy extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private OpCommon opCommon;
    private RobotMap robotMap;
    private SequentialCommandGroup temp;

    /**
     * Trajectories
     */
    private TrajectorySequenceBuilder
        traj;

    public void init_traj() {
        traj = drive.trajectorySequenceBuilder(new Pose2d())
            .turn(Math.toRadians(135));
    }

    /**
     * La Program
     */
    @Override
    public void initialize() {
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.AUTO);
        drive = new SampleMecanumDrive(robotMap);
        opCommon = new OpCommon(robotMap, RobotEx.Alliance.RED);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        temp = opCommon.specimenIntake();
        temp.schedule();
        init_traj();
        drive.followTrajectorySequenceAsync(traj.build());
        while (
            !isStopRequested()
                && opModeIsActive()
//                && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            drive.update();
            run();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        PoseStorage.currentPose = current_pose;
    }
}
