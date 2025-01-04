package org.firstinspires.ftc.teamcode.IntoTheDeepRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.CouplersSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.drive.DriveConstants;

public class IntoTheDeepRobot extends RobotEx {
    protected RobotMap robotMap;
    //----------------------------------- Initialize Subsystems ----------------------------------//
    protected ClawSubsystem clawSubsystem;
    protected ArmSubsystem armSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected CouplersSubsystem couplersSubsystem;

    public IntoTheDeepRobot(RobotMap robotMap, DriveConstants RobotConstants,
                            OpModeType opModeType, Alliance alliance, boolean init_camera,
                            Pose2d startingPose, Telemetry telemetry) {
        super(robotMap, RobotConstants, opModeType, alliance, init_camera, startingPose, telemetry);
        this.robotMap = robotMap;
        this.initMechanismsTeleOp();
    }

    @Override
    public void initMechanismsTeleOp() {
//        clawSubsystem = new ClawSubsystem(this.robotMap);
//        armSubsystem = new ArmSubsystem(this.robotMap);
//        intakeSubsystem = new IntakeSubsystem(this.robotMap);
        couplersSubsystem = new CouplersSubsystem(this.robotMap);


    }
}
