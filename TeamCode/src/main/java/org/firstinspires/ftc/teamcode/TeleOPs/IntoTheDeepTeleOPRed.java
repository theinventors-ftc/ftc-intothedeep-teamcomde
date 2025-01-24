package org.firstinspires.ftc.teamcode.TeleOPs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.RobotMapInterface;

@TeleOp(name = "TeleOP RED", group = "Final TeleOPs")
public class IntoTheDeepTeleOPRed extends TeleOpBase {
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
