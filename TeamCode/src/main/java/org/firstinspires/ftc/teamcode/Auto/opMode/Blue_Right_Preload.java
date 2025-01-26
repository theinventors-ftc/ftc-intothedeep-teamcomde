package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.function.DoubleSupplier;

@Autonomous(name = "Blue Right Preload")
public class Blue_Right_Preload extends CommandOpMode {

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
            -Tile + robotX/2, (3 * Tile) - robotY/2, Math.toRadians(90)
        ),

        preload = new Pose2d(
            -0.1 * Tile , 1.1 * Tile + (robotY/2), Math.toRadians(90)
        ),

        parking = new Pose2d(
            -2 * Tile, 2.5 * Tile, Math.toRadians(135)
        );

    /**
     * Trajectories
     */
    private TrajectorySequenceBuilder
        toPreload,
        toParking;

    public void init_toPreload() {
        toPreload = drive.trajectorySequenceBuilder(startPose)
            .waitSeconds(0.5)
            .strafeTo(preload.vec());
    }
    public void init_toParking() {
        toParking = drive.trajectorySequenceBuilder(current_pose)
            .setTangent(Math.toRadians(90))
            .splineTo(parking.vec(), Math.toRadians(135));
    }

    /**
     * La Program
     */
    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, true);
        opCommon = new OpCommon(robotMap);
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
                && drive.isBusy()
        ) {
            drive.update();
            run();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = opCommon.reset_elevator();
        temp.schedule();
        init_toParking();
        drive.followTrajectorySequenceAsync(toParking.build());
        while (
            !isStopRequested()
                && opModeIsActive()
                && drive.isBusy()
        ) {
            drive.update();
            run();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

//        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
