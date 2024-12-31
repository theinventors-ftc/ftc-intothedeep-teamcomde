package org.firstinspires.ftc.teamcode.IntoTheDeepRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;

public class IntoTheDeepRobot extends RobotEx {
    //----------------------------------- Initialize Subsystems ----------------------------------//

    public IntoTheDeepRobot(HardwareMap hm, DriveConstants RobotConstants, Telemetry telemetry,
                            GamepadExEx driverOp, GamepadExEx toolOp, OpModeType opModeType,
                            Alliance alliance, String imuName, boolean init_camera,
                            Pose2d startingPose){
        super(hm, RobotConstants, telemetry, driverOp, toolOp, opModeType, alliance, imuName,
                init_camera,false, startingPose);
    }

    @Override
    public void initMechanismsAutonomous(HardwareMap hardwareMap) {
        super.initMechanismsAutonomous(hardwareMap);
    }

    @Override
    public void initMechanismsTeleOp(HardwareMap hardwareMap) {
        super.initMechanismsTeleOp(hardwareMap);
    }
}
