package org.firstinspires.ftc.teamcode.Auto.opMode;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotX;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.tipPoseTransfer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Auto.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

@Autonomous(name = "Auto5Samples", group = "Special")
public class AutoStarlight_5Samples extends CommandOpMode {

    private SampleMecanumDrive drive;
    private volatile Pose2d current_pose;
    private DoubleSupplier extendo_length = ()-> 0.0;
    private OpCommon opCommon;
    private RobotMap robotMap;
    private SequentialCommandGroup temp;
    private Timing.Timer timer;

    /**
     * Poses
     */
    private Pose2d

        startPose = new Pose2d(
            -Tile + robotX/2, (-3 * Tile) + robotY/2, Math.toRadians(270)
        ),

        chambers = new Pose2d(
            -0.25 * Tile, -1.1 * Tile - (robotY/2), Math.toRadians(270)
        ),

        basket = new Pose2d(
            -2.55 * Tile, -1.85 * Tile, Math.toRadians(50)
        ),

        neutralSampleRight = new Pose2d(
            -2 * Tile - 2, -Tile - 2, Math.toRadians(90)
        ),

        neutralSampleMid = tipPoseTransfer(new Pose2d(
            -2.5 * Tile-1, -Tile, Math.toRadians(90)
        ), extendo_length.getAsDouble()),

        neutralSampleLeft = new Pose2d(
            -2.3 * Tile, -1.5 * Tile + 2.3, Math.toRadians(159) // PEOS
        ),

        submersible = new Pose2d(
                -Tile+2.5, -0.5 * Tile, Math.toRadians(0)
        ),

        parking = new Pose2d(
            -1.25 * Tile+2, -0.5 * Tile-3, Math.toRadians(0)
        );

    /**
     * Trajectories
     */
    private TrajectorySequenceBuilder
        toPreload,
        toNeutral_0,
        toNeutral_1,
        toNeutral_2,
        toBasketExtra,
        toBasketFinal,
        toBasketBlind,
        toBasket_0,
        toBasket_1,
        toSubmersible,
        toParking;

    public void init_toPreload() {
        toPreload = drive.trajectorySequenceBuilder(startPose)
            .setReversed(true)
            .setTangent(Math.toRadians(135))
            .splineToSplineHeading(new Pose2d(basket.getX() + 3.5, basket.getY(),
                                              Math.toRadians(68)), Math.toRadians(200));
    }
    public void init_toNeutral_0() {
        toNeutral_0 = drive.trajectorySequenceBuilder(current_pose)
                .lineToLinearHeading(new Pose2d(basket.getX() + 3.5, basket.getY(), Math.toRadians(68)));
    }
    public void init_toNeutral_1() {
        toNeutral_1 = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(new Pose2d(basket.getX()-0.5, basket.getY() + 2, Math.toRadians(83)));
    }
    public void init_toNeutral_2() {
        toNeutral_2 = drive.trajectorySequenceBuilder(current_pose)
            .lineToLinearHeading(neutralSampleLeft);
    }
    public void init_toBasket_0() {
        toBasket_0 = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .setTangent(Math.toRadians(315))
            .splineToSplineHeading(new Pose2d(basket.getX(), basket.getY() + 5,
                                             basket.getHeading()), Math.toRadians(180));
    }
    public void init_toBasketExtra() {
        toBasketExtra = drive.trajectorySequenceBuilder(current_pose)
                .setReversed(true)
                .setTangent(Math.toRadians(315))
                .splineToSplineHeading(new Pose2d(basket.getX(), basket.getY() + 5,
                        Math.toRadians(50)), Math.toRadians(180));
    }
    public void init_toBasket_1() {
        toBasket_1 = drive.trajectorySequenceBuilder(current_pose)
            .setReversed(true)
            .strafeTo(basket.vec());
    }
    public void init_toBasketFinal() {
        toBasketFinal = drive.trajectorySequenceBuilder(current_pose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(basket.getX()-2, basket.getY() + 1,//5
                        Math.toRadians(70)));
    }

    public void init_toBasketBlind() {
        toBasketBlind = drive.trajectorySequenceBuilder(current_pose)
                .setReversed(true)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(basket.getX()+1, basket.getY() + 3,
                        basket.getHeading()), Math.toRadians(250));
    }

    public void init_toSubmersible() {
        toSubmersible = drive.trajectorySequenceBuilder(current_pose)
                .splineTo(submersible.vec(), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(70,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(55)
                )
                .forward(1.5);
//                .splineToLinearHeading(submersible, Math.toRadians(0));
    }

    public void init_toParking() {
        toParking = drive.trajectorySequenceBuilder(current_pose)
            .splineTo(parking.vec(), Math.toRadians(45),
                    SampleMecanumDrive.getVelocityConstraint(70,
                            DriveConstants.MAX_ANG_VEL,
                            DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(55)
            );
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
        timer = new Timing.Timer(30000, TimeUnit.MILLISECONDS);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        temp = new SequentialCommandGroup(
                new WaitCommand(2000),
                new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(400)), // Prep Extendo
                opCommon.basket_scoring()
        );
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
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = opCommon.release_sample();
        temp.schedule();
        while (
                !isStopRequested()
                        && opModeIsActive()
                        && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

        /* -----0----- */

        temp = new SequentialCommandGroup(
            opCommon.extendo(0.6),
            opCommon.sample_intake()
        );
        temp.schedule();
        init_toNeutral_0();
        drive.followTrajectorySequenceAsync(toNeutral_0.build());
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
                new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(150)),
                new WaitCommand(200),
                new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(350)), // Prep Extendo
                opCommon.basket_scoring()
        );
        temp.schedule();
        while (
            !isStopRequested()
            && opModeIsActive()
            && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

        /* -----1----- */

        temp = new SequentialCommandGroup(
            opCommon.release_sample(),
            new WaitCommand(1000),
            opCommon.extendo(0.4),
            opCommon.sample_intake()
        );
        temp.schedule();
        init_toNeutral_1();
        drive.followTrajectorySequenceAsync(toNeutral_1.build());
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

        temp = new SequentialCommandGroup(
                new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(150)),
                new WaitCommand(200),
                new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(400)), // Prep Extendo
                opCommon.basket_scoring(),
                opCommon.release_sample()
        );
        temp.schedule();
        while (
            !isStopRequested()
                && opModeIsActive()
                && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }

//        basket = new Pose2d(basket.getX() - 2, basket.getY(), basket.getHeading());
//        init_toBasket_1();
//        drive.followTrajectorySequenceAsync(toBasket_1.build());
//        while (
//            !isStopRequested()
//                && opModeIsActive()
//                && drive.isBusy()
//        ) {
//            drive.update();
//        }
//        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
//        current_pose = drive.getPoseEstimate();

//        temp = opCommon.release_sample();
//        temp.schedule();
//        while (
//            !isStopRequested()
//            && opModeIsActive()
//            && CommandScheduler.getInstance().isScheduled(temp)
//        ) {
//            run();
//        }

        /*-- 2 --*/
        temp = new SequentialCommandGroup(
            new WaitCommand(1000),
            opCommon.extendo3dSample(0.35),
            opCommon.sample_intake()
        );
        temp.schedule();
        init_toNeutral_2();
        drive.followTrajectorySequenceAsync(toNeutral_2.build());
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

        temp = new SequentialCommandGroup(
                new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(150)),
                new WaitCommand(200),
                new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(400)), // Prep Extendo
                opCommon.basket_scoring()
        );
        temp.schedule();
        basket = new Pose2d(basket.getX() + 5, basket.getY(), Math.toRadians(75));
        init_toBasketFinal();
        drive.followTrajectorySequenceAsync(toBasketFinal.build());
        while (
            !isStopRequested()
            && opModeIsActive()
            && drive.isBusy()
            && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            drive.update();
            run();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        current_pose = drive.getPoseEstimate();

        temp = opCommon.release_sample();
        temp.schedule();
        while (
            !isStopRequested()
                && opModeIsActive()
                && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }
        /* -----S----- */
        temp = new SequentialCommandGroup(
                new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(500)), // Prep Extendo
                opCommon.reset_elevator()
        );
        temp.schedule();
        init_toSubmersible();
        drive.followTrajectorySequenceAsync(toSubmersible.build());
        while (
                !isStopRequested()
                        && opModeIsActive()
                        && (drive.isBusy()
                        || CommandScheduler.getInstance().isScheduled(temp))
        ) {
            drive.update();
            run();
        }
        drive.setPoseEstimate(new Pose2d(-14.3, drive.getPoseEstimate().getY(), Math.toRadians(0)));
        current_pose = drive.getPoseEstimate();

        temp = new SequentialCommandGroup(
                opCommon.deep_intake(),
                opCommon.submersible_intake()
        );
        temp.schedule();
        timer.start();

        boolean sub_sample_taken = true;
        while (
                !isStopRequested()
                        && opModeIsActive()
                        && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();

            if (timer.elapsedTime() >= 4000) {
                sub_sample_taken = false;
                break;
            }
        }

        // Blind Sample
        if (sub_sample_taken) {
            temp = new SequentialCommandGroup(
                    new InstantCommand(() -> opCommon.intakeSubsystem.reverse()),
                    new WaitCommand(150),
                    new InstantCommand(() -> opCommon.intakeSubsystem.stop()),
                    new ParallelCommandGroup(
                            opCommon.basket_scoring(),
                            new SequentialCommandGroup(
                                    new WaitCommand(500),
                                    new InstantCommand(() -> opCommon.extendoSubsystem.set_MAX_POWER(1)),
                                    new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(500)) // Prep Extendo
                            )
                    )
            );
            temp.schedule();
            init_toBasketBlind();
            drive.followTrajectorySequenceAsync(toBasketBlind.build());
            while (!isStopRequested()
                    && opModeIsActive()
                    && (drive.isBusy()
                    || CommandScheduler.getInstance().isScheduled(temp))) {
                drive.update();
                run();
            }
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
            current_pose = drive.getPoseEstimate();

//        init_toBasket_1();
//        drive.followTrajectorySequenceAsync(toBasket_1.build());
//        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
//            drive.update();
//            run();
//        }
//        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
//        current_pose = drive.getPoseEstimate();

            temp = new SequentialCommandGroup(
                    opCommon.release_sample(),
                    opCommon.reset_elevator()
            );
            temp.schedule();
            while (
                    !isStopRequested()
                            && opModeIsActive()
                            && CommandScheduler.getInstance().isScheduled(temp)
            ) {
                run();
            }


            /* -----P----- */

            temp = new SequentialCommandGroup(
                    new InstantCommand(() -> opCommon.extendoSubsystem.setTargetPosition(-50)),
                    new InstantCommand(() -> opCommon.elevatorSubsystem.set_target_height(-50))
            );
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
        } else {
            temp = new SequentialCommandGroup(
                    new InstantCommand(opCommon.intakeSubsystem::raise),
                    new InstantCommand(opCommon.intakeSubsystem::stop),
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
                    new WaitUntilCommand(() -> opCommon.extendoSubsystem.atTarget())
            );
            temp.schedule();
            while (
                    !isStopRequested()
                            && opModeIsActive()
                            && CommandScheduler.getInstance().isScheduled(temp)
            ) {
                run();
            }
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}