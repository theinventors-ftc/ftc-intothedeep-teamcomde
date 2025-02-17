package org.firstinspires.ftc.teamcode.Tests;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.ForwardControllerSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.HeadingControllerSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.StrafeControllerSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.CouplersSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.DistanceSensorsSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.drive.MecanumDriveCommand;
import org.inventors.ftc.robotbase.drive.MecanumDriveSubsystem;

@Autonomous(name = "Control Test", group = "Tests")
public class ControlTest extends CommandOpMode {

    private SequentialCommandGroup temp;
    private SampleMecanumDrive drive;
    private RobotMap robotMap;
    private FtcDashboard dashboard;

    private double[] pidaTargets = {44.615, 10, 75};

    /*--Subsystems--*/
    //    protected HardwareMap hardwareMap;
    public ClawSubsystem clawSubsystem;
    public ArmSubsystem armSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public CouplersSubsystem couplersSubsystem;

    /*--Controllers--*/
    private ForwardControllerSubsystem forwardControllerSubsystem;
    private StrafeControllerSubsystem strafeControllerSubsystem;
    private HeadingControllerSubsystem gyroFollow;
    private DistanceSensorsSubsystem distanceSensorsSubsystem;

    private SequentialCommandGroup activateDistanceCalibration() {
        return new SequentialCommandGroup(
            new InstantCommand(gyroFollow::enable),
            new InstantCommand(strafeControllerSubsystem::enable),
            new InstantCommand(forwardControllerSubsystem::enable),
            new InstantCommand(() -> gyroFollow.setGyroTarget(pidaTargets[2])),
            new InstantCommand(() -> strafeControllerSubsystem.setDistTarget(pidaTargets[1])),//3
            // .555
            new InstantCommand(() -> forwardControllerSubsystem.setGyroTarget(pidaTargets[0]))
        );
    }

    private SequentialCommandGroup deactivateDistanceCalibration() {
        return new SequentialCommandGroup(
            new InstantCommand(gyroFollow::disable),
            new InstantCommand(strafeControllerSubsystem::disable),
            new InstantCommand(forwardControllerSubsystem::disable)
        );
    }

    private void init_controllers(SampleMecanumDrive drive) {
        this.dashboard = FtcDashboard.getInstance();
        forwardControllerSubsystem = new ForwardControllerSubsystem(
            () -> distanceSensorsSubsystem.getDistances()[0],
            dashboard.getTelemetry()
        );
        strafeControllerSubsystem = new StrafeControllerSubsystem(
            () -> distanceSensorsSubsystem.getDistances()[1], // TODO Change to 2
            dashboard.getTelemetry()
        );
        gyroFollow = new HeadingControllerSubsystem(
            () -> Math.toDegrees(drive.getPoseEstimate().getHeading()),
            () -> 0,
            dashboard.getTelemetry()
        );
    }

    private TrajectorySequence traj;

    private double drivetrainStrafe() {
        return strafeControllerSubsystem.calculatePower();
    }

    private double drivetrainForward() {
        return forwardControllerSubsystem.calculatePower();
    }

    private double drivetrainTurn() {
        return -gyroFollow.calculateTurn();
    }

    public void robotCentricMovement(double x, double y, double t) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(y) + abs(x) + abs(t), 1);
        double frontLeftPower = (y + x + t) / denominator;
        double backLeftPower = (y - x + t) / denominator;
        double frontRightPower = (y - x - t) / denominator;
        double backRightPower = (y + x - t) / denominator;

        robotMap.getFrontLeftMotor().set(frontLeftPower);
        robotMap.getRearLeftMotor().set(backLeftPower);
        robotMap.getFrontRightMotor().set(frontRightPower);
        robotMap.getRearRightMotor().set(backRightPower);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.AUTO);
        clawSubsystem = new ClawSubsystem(this.robotMap);
        armSubsystem = new ArmSubsystem(this.robotMap);
        intakeSubsystem = new IntakeSubsystem(this.robotMap);
        elevatorSubsystem = new ElevatorSubsystem(
            this.robotMap,
            () -> 0.0,
            robotMap.getRearLeftMotor(),
            robotMap.getTelemetry(),
            false
        );
        couplersSubsystem = new CouplersSubsystem(this.robotMap);
        distanceSensorsSubsystem = new DistanceSensorsSubsystem(
            this.robotMap,
            robotMap.getTelemetry()
        );

        drive = new SampleMecanumDrive(robotMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
        init_controllers(drive);
    }

    @Override
    public void runOpMode() {
        initialize();

        traj = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(90)))
            .turn(Math.toRadians(50))
            .build();

        waitForStart();

        drive.followTrajectorySequenceAsync(traj);
        while (
            !isStopRequested()
                && opModeIsActive()
                && drive.isBusy()
        ) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        //
        drive.deactivate();
        //
        temp = activateDistanceCalibration();
        temp.schedule();
        while (
            !isStopRequested()
                && opModeIsActive()
//                && !(distanceSensorsSubsystem.getDistances()[0] <= 46
//                && distanceSensorsSubsystem.getDistances()[0] >= 42)
//                && !(distanceSensorsSubsystem.getDistances()[1] <= 4)
//                && !(-gyroFollow.calculateTurn() <= 80
//                && -gyroFollow.calculateTurn() >= 70)
        ) {
            robotCentricMovement(drivetrainStrafe(), drivetrainForward(), drivetrainTurn());
            drive.update();
            run();
        }

        temp = deactivateDistanceCalibration();
        temp.schedule();
        while (
            !isStopRequested()
                && opModeIsActive()
                && CommandScheduler.getInstance().isScheduled(temp)
        ) {
            run();
        }
    }
}
