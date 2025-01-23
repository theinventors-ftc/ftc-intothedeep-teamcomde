package org.firstinspires.ftc.teamcode.Auto.test;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotMap;

@Autonomous(name = "Gyro Test")
public class gyrotest extends LinearOpMode {

    private RobotMap robotMap;

    @Override
    public void runOpMode() {

        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            telemetry.addData("Angular Velocity",
                              (double) robotMap.getIMU()
                                  .getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate);
            telemetry.update();
        }
    }
}
