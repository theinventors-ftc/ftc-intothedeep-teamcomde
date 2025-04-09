package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class IntakeV2Subsystem extends SubsystemBase {
    public ServoImplEx raiseServoL, raiseServoR, arm_rot, arm_lift, claw_rot, claw;
    private Telemetry telemetry;

    // ------------------------------------------ States ---------------------------------------- //

    public enum RaiseState { INTAKE, PASSTHROUGH }
    public enum ArmRotState { INTAKE, SAMPLE_FOLLOW, PASSTHROUGH }
    public enum ArmLiftState { INTAKE_AIM, INTAKE, PASSTHROUGH }
    public enum ClawRotState { INTAKE, SAMPLE_FOLLOW, PASSTHROUGH }
    public enum ClawState { RELEASED, GRIPPED }

    private RaiseState raiseState;
    private ArmRotState armRotState;
    private ArmLiftState armLiftState;
    private ClawRotState clawRotState;
    private ClawState clawState;

    private final HashMap<RaiseState, Double> raise_positions = new HashMap<RaiseState, Double>() {{ // done
        put(RaiseState.INTAKE, 0.835);
        put(RaiseState.PASSTHROUGH, 0.17);
    }};
    private final HashMap<ArmRotState, Double> arm_rot_positions = new HashMap<ArmRotState, Double>() {{ // done
        put(ArmRotState.INTAKE, 0.185);
        put(ArmRotState.SAMPLE_FOLLOW, 0.185);
        put(ArmRotState.PASSTHROUGH, 0.85);
    }};
    private final HashMap<ArmLiftState, Double> arm_lift_positions = new HashMap<ArmLiftState, Double>() {{ //
        put(ArmLiftState.INTAKE_AIM, 0.4);
        put(ArmLiftState.INTAKE, 0.36);
        put(ArmLiftState.PASSTHROUGH, 0.0);
    }};
    private final HashMap<ClawRotState, Double> claw_rot_positions = new HashMap<ClawRotState, Double>() {{
        put(ClawRotState.INTAKE, 0.43);
        put(ClawRotState.SAMPLE_FOLLOW, 0.43);
        put(ClawRotState.PASSTHROUGH, 0.43);
    }};
    private final HashMap<ClawState, Double> claw_positions = new HashMap<ClawState, Double>() {{
        put(ClawState.RELEASED, 0.0);
        put(ClawState.GRIPPED, 0.5);
    }};

    // ------------------------------------------------------------------------------------------ //
    public IntakeV2Subsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        raiseServoL = hardwareMap.get(ServoImplEx.class, "raise_servo_l");
        raiseServoR = hardwareMap.get(ServoImplEx.class, "raise_servo_r");
        arm_rot = hardwareMap.get(ServoImplEx.class, "arm_rot");
        arm_lift = hardwareMap.get(ServoImplEx.class, "arm_lift");
        claw_rot = hardwareMap.get(ServoImplEx.class, "claw_rot");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        setRaiseState(RaiseState.PASSTHROUGH);
        setArmRotState(ArmRotState.PASSTHROUGH);
        setArmLiftState(ArmLiftState.PASSTHROUGH);
        setClawRotState(ClawRotState.PASSTHROUGH);
        setClawState(ClawState.GRIPPED);
    }

    // ---------------------------------------- Actuators --------------------------------------- //
    public void setRaiseState(RaiseState state) {
        this.raiseState = state;
        raiseServoL.setPosition(raise_positions.get(state));
        raiseServoR.setPosition(1.0-raise_positions.get(state));
    }

    public void setArmRotState(ArmRotState state) {
        this.armRotState = state;
        if(state == ArmRotState.SAMPLE_FOLLOW) return;
        arm_rot.setPosition(arm_rot_positions.get(state));
    }

    public void setArmRotation(double position) {
        if (armRotState != ArmRotState.SAMPLE_FOLLOW) return;

        arm_rot.setPosition(Range.clip(position, 0.0, 0.185*2)); // Center: 0.185, Limit: 0.45
    }

    public void setArmLiftState(ArmLiftState state) {
        this.armLiftState = state;
        arm_lift.setPosition(arm_lift_positions.get(state));
    }

    public void setClawRotState(ClawRotState state) {
        this.clawRotState = state;
        if(state == ClawRotState.SAMPLE_FOLLOW) return;
        claw_rot.setPosition(claw_rot_positions.get(state));
    }

    public void setClawRotation(double position) {
        if(clawRotState != ClawRotState.SAMPLE_FOLLOW) return;
        claw_rot.setPosition(Range.clip(position, 0.0, 1.0));
    }

    public void setClawState(ClawState state) {
        this.clawState = state;
        claw.setPosition(claw_positions.get(state));
    }

    // -------------------------------------- State Getters ------------------------------------- //
    public RaiseState getRaiseState() {
        return raiseState;
    }

    public ArmRotState getArmRotState() {
        return armRotState;
    }

    public ArmLiftState getArmLiftState() {
        return armLiftState;
    }

    public ClawRotState getClawRotState() {
        return clawRotState;
    }

    public ClawState getClawState() {
        return clawState;
    }
}
