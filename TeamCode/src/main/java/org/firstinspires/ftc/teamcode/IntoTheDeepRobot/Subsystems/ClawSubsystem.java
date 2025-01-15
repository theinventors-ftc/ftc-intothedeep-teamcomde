package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.HashMap;

public class ClawSubsystem extends SubsystemBase {
    private final ServoImplEx clawServo, clawRotServo;

    // --------------------------------------- Claw States -------------------------------------- //
    public enum ClawState {
        OPEN,
        JUST_OPENED,
        CLOSED
    }
    public enum ClawRotState {
        NORMAL,
        FLIPPED,
    }
    private ClawState state;
    private ClawRotState rotState;

    private final HashMap<ClawState, Double> claw_positions = new HashMap<ClawState, Double>() {{
        put(ClawState.CLOSED, 0.8);
        put(ClawState.JUST_OPENED, 0.65);
        put(ClawState.OPEN, 0.5);
    }};

    private final HashMap<ClawRotState, Double> claw_rot_positions = new HashMap<ClawRotState, Double>() {{
        put(ClawRotState.NORMAL, 0.06);
        put(ClawRotState.FLIPPED, 0.4);
    }};

    public ClawSubsystem(RobotMap robotMap) {
        clawServo = robotMap.getClawServo();
        clawRotServo = robotMap.getClawRotServo();

        goNormal();
        justOpen();
    }

    // ---------------------------------------- Actuators --------------------------------------- //
    public void grab() {
        this.state = ClawState.CLOSED;
        clawServo.setPosition((double)claw_positions.get(ClawState.CLOSED));
    }

    public void justOpen() {
        this.state = ClawState.JUST_OPENED;
        clawServo.setPosition((double)claw_positions.get(ClawState.JUST_OPENED));
    }

    public void release() {
        this.state = ClawState.OPEN;
        clawServo.setPosition((double)claw_positions.get(ClawState.OPEN));
    }

    public void goNormal() {
        this.rotState = ClawRotState.NORMAL;
        clawRotServo.setPosition((double)claw_rot_positions.get(ClawRotState.NORMAL));
    }

    public void goFlipped() {
        this.rotState = ClawRotState.FLIPPED;
        clawRotServo.setPosition((double)claw_rot_positions.get(ClawRotState.FLIPPED));
    }

    // -------------------------------------- State Getters ------------------------------------- //
    public ClawState getState() {
        return state;
    }

    public ClawRotState getRotState() {
        return rotState;
    }
}
