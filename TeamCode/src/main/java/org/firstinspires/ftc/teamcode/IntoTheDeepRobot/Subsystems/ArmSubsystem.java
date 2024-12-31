package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;

public class ArmSubsystem extends SubsystemBase {
    private final ServoImplEx armLeft, armRight, wrist;

    enum ArmState {
        UP,
        DOWN
    }
    enum WristState {
        UP,
        DOWN
    }
    private ArmState armState;
    private WristState wristState;

    HashMap<ArmState, Double> arm_positions = new HashMap<ArmState, Double>() {{
        put(ArmState.UP, 0.0);
        put(ArmState.DOWN, 0.5);
    }};
    public ArmSubsystem(HardwareMap hm) {
        armLeft = hm.get(ServoImplEx.class, "arm_left");
        armRight = hm.get(ServoImplEx.class, "arm_right");
        wrist = hm.get(ServoImplEx.class, "wrist");
    }
}
