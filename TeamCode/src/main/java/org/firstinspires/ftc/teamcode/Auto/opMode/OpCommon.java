package org.firstinspires.ftc.teamcode.Auto.opMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OpCommon extends CommandBase {

    public static SequentialCommandGroup temp;

    //TODO: Make class

    /**
     * Commands and Subsystems for mechanisms
     */
    public static Pose2d fixedPose2d(Pose2d pose) {
        return new Pose2d(pose.vec(), Math.toRadians(pose.getHeading()));
    }

    /**
     * Initialization of all subsystems and mechanisms
     */
    public static void init_mechanisms(HardwareMap hm, Telemetry tele) {

    }
}
