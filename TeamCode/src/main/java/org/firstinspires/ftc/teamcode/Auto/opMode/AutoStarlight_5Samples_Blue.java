package org.firstinspires.ftc.teamcode.Auto.opMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.inventors.ftc.robotbase.RobotEx;

@Autonomous(name = "Auto5SamplesBLUE", group = "Special")
public class AutoStarlight_5Samples_Blue extends AutoStarlight_5Samples {
    @Override
    public void initialize() {
        setAlliance(RobotEx.Alliance.BLUE);
        super.initialize();
    }

    @Override
    public void run() {
        super.run();
    }
}
