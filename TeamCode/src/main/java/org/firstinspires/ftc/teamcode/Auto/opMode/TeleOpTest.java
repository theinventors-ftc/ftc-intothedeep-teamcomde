package org.firstinspires.ftc.teamcode.Auto.opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOPs.TeleOpBase;
import org.inventors.ftc.robotbase.RobotEx;

@TeleOp(name = "TeleOP TEST", group = "Final TeleOPs")
public class TeleOpTest extends autoInTeleOp {
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
