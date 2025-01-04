package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.inventors.ftc.robotbase.RobotEx;

@TeleOp(name = "TeleOP BLUE", group = "Final TeleOPs")
public class IntoTheDeepTeleOPBlue extends TeleOpBase {
    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(RobotEx.Alliance.BLUE);
    }

    @Override
    public void run() {
        super.run();
    }
}
