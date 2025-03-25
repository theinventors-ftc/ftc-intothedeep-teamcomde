package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.HashMap;

public class ClawSubsystem extends SubsystemBase {
    private final ServoImplEx clawServo;

    // --------------------------------------- Claw States -------------------------------------- //
    public enum ClawState {
        OPEN,
        JUST_OPENED,
        FIRMLY_GRIPPED,
        LOOSLY_GRIPPED,
    }

    private ClawState state;

    private final HashMap<ClawState, Double> claw_positions = new HashMap<ClawState, Double>() {{
        put(ClawState.OPEN, 0.4);
        put(ClawState.JUST_OPENED, 0.22);
        put(ClawState.FIRMLY_GRIPPED, 0.05);
        put(ClawState.LOOSLY_GRIPPED, 0.115);
    }};

    public ClawSubsystem(RobotMap robotMap) {
        clawServo = robotMap.getClawServo();
        firmlyGripped();
    }

    // ---------------------------------------- Actuators --------------------------------------- //
    public void release() {
        this.state = ClawState.OPEN;
        clawServo.setPosition((double)claw_positions.get(ClawState.OPEN));
    }

    public void justOpen() {
        this.state = ClawState.JUST_OPENED;
        clawServo.setPosition((double)claw_positions.get(ClawState.JUST_OPENED));
    }

    public void firmlyGripped() {
        this.state = ClawState.FIRMLY_GRIPPED;
        clawServo.setPosition((double)claw_positions.get(ClawState.FIRMLY_GRIPPED));
    }

    public void looslyGripped() {
        this.state = ClawState.LOOSLY_GRIPPED;
        clawServo.setPosition((double)claw_positions.get(ClawState.LOOSLY_GRIPPED));
    }

    // -------------------------------------- State Getters ------------------------------------- //
    public ClawState getState() {
        return state;
    }
}
