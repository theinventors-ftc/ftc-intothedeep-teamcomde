package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.HashMap;

public class ClawSubsystem extends SubsystemBase {
    private final ServoImplEx clawServo, clawRotServo;

    // --------------------------------------- Claw States -------------------------------------- //
    enum ClawState {
        OPEN,
        JUST_OPENED,
        CLOSED
    }
    enum ClawRotState {
        NORMAL,
        FLIPPED,
    }
    private ClawState state;
    private ClawRotState rotState;

    private HashMap<ClawState, Double> claw_positions = new HashMap<ClawState, Double>() {{
        put(ClawState.CLOSED, 0.2);
        put(ClawState.JUST_OPENED, 0.5);
        put(ClawState.OPEN, 0.8);
    }}; // TODO: Find the correct positions

    private HashMap<ClawRotState, Double> claw_rot_positions = new HashMap<ClawRotState, Double>() {{
        put(ClawRotState.NORMAL, 0.1);
        put(ClawRotState.FLIPPED, 0.9);
    }}; // TODO: Find the correct positions

    public ClawSubsystem(HardwareMap hm) {
        clawServo = hm.get(ServoImplEx.class, "claw");
        clawRotServo = hm.get(ServoImplEx.class, "claw_rot");

        goNormal();
        grab();
    }

    // ------------------------------------------ Claw ------------------------------------------ //
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

    //--------------------------------------- Claw Rotation --------------------------------------//
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
