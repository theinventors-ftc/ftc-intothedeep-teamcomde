package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.inventors.ftc.robotbase.RobotEx;

@TeleOp(name = "TeleOP RED", group = "Final TeleOPs")
public class TeleOpRedAlliance extends TeleOpBase {
    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(RobotEx.Alliance.RED);
    }

    @Override
    public void run() {
        super.run();
    }
}