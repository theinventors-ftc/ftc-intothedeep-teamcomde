package org.firstinspires.ftc.teamcode.TeleOPs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.IntoTheDeepRobot;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;

@Disabled
@TeleOp(name = "Do not run this TeleOP", group = "")
public class TeleOpBase extends CommandOpMode {
    GamepadExEx driverOp, toolOp;
    private DriveConstants RobotConstants;
    public Pose2d pose;
    private ElapsedTime runtime;
    private IntoTheDeepRobot robot;

    private RobotMap robotMap;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.TELEOP);

        // ----------------------------------- Robot Constants ---------------------------------- //
        RobotConstants = new DriveConstants();

        RobotConstants.frontLeftInverted = true;
        RobotConstants.frontRightInverted = true;
        RobotConstants.rearRightInverted = true;
        RobotConstants.rearLeftInverted = true;

        RobotConstants.WHEEL_RADIUS = 1; // inch
        RobotConstants.GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        RobotConstants.TRACK_WIDTH = 10; // in

        RobotConstants.MAX_VEL = 90;
        RobotConstants.MAX_ACCEL = 90;
        RobotConstants.MAX_ANG_VEL = Math.toRadians(360);
        RobotConstants.MAX_ANG_ACCEL = Math.toRadians(360);

        RobotConstants.RUN_USING_ENCODER = false;

        RobotConstants.frontLeftFeedForward[0] = 0;
        RobotConstants.frontLeftFeedForward[1] = 1;
        RobotConstants.frontLeftFeedForward[2] = 0;
        RobotConstants.frontRightFeedForward[0] = 0;
        RobotConstants.frontRightFeedForward[1] = 1;
        RobotConstants.frontRightFeedForward[2] = 0;
        RobotConstants.rearLeftFeedForward[0] = 0;
        RobotConstants.rearLeftFeedForward[1] = 1;
        RobotConstants.rearLeftFeedForward[2] = 0;
        RobotConstants.rearRightFeedForward[0] = 0;
        RobotConstants.rearRightFeedForward[1] = 1;
        RobotConstants.rearRightFeedForward[2] = 0;

        RobotConstants.VELO_KP = 0;
        RobotConstants.VELO_KI = 0;
        RobotConstants.VELO_KD = 0;

        RobotConstants.TICKS_PER_REV = 537;
        RobotConstants.MAX_RPM = 435;

        RobotConstants.DEFAULT_SPEED_PERC = 0.65;
        RobotConstants.SLOW_SPEED_PERC = 0.2;
        RobotConstants.FAST_SPEED_PERC = 1;

        // ---------------------------- Transfer Pose from Autonomous --------------------------- //
        pose = PoseStorage.currentPose;
    }

    public void initAllianceRelated(RobotEx.Alliance alliance) {
        robot = new IntoTheDeepRobot(robotMap, RobotConstants, RobotEx.OpModeType.TELEOP, alliance,
                false, pose);
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }
}