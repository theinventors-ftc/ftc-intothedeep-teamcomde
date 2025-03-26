package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.HashMap;

public class ArmSubsystem extends SubsystemBase {
    private final ServoImplEx armLeft, armRight, wrist;

    // ------------------------------------------ States ---------------------------------------- //
    public enum ArmState {
        INTAKE,
        PARK,
        BASKET_OUTTAKE,
        LVL1_ASCENT,
        SPEC_INTAKE_NEW,
        SPEC_OUTTAKE_AIM_NEW,
        SPEC_OUTTAKE_NEW
    }
    public enum WristState {
        INTAKE,
        PARK,
        BASKET_OUTTAKE,
        LVL1_ASCENT,
        SPEC_INTAKE_NEW,
        SPEC_OUTTAKE_AIM_NEW,
        SPEC_OUTTAKE_NEW
    }
    private ArmState armState;
    private WristState wristState;

    private final HashMap<ArmState, Double> arm_positions = new HashMap<ArmState, Double>() {{
        put(ArmState.INTAKE, 0.093);
        put(ArmState.PARK, 0.23);
        put(ArmState.BASKET_OUTTAKE, 0.46);
        put(ArmState.LVL1_ASCENT, 0.5);
        put(ArmState.SPEC_INTAKE_NEW, 0.13);
        put(ArmState.SPEC_OUTTAKE_AIM_NEW, 0.56);
        put(ArmState.SPEC_OUTTAKE_NEW, 0.4);
    }};

    private final HashMap<WristState, Double> wrist_positions = new HashMap<WristState, Double>() {{
        put(WristState.INTAKE, 0.195);
        put(WristState.PARK, 0.1);
        put(WristState.BASKET_OUTTAKE, 0.95);
        put(WristState.LVL1_ASCENT, 0.78);
        put(WristState.SPEC_INTAKE_NEW, 0.6);
        put(WristState.SPEC_OUTTAKE_AIM_NEW, 0.5);
        put(WristState.SPEC_OUTTAKE_NEW, 0.44);
    }};

    public ArmSubsystem(RobotMap robotMap) {
        armLeft = robotMap.getArmLeftServo();
        armRight = robotMap.getArmRightServo();
        wrist = robotMap.getArmWristServo();

        setArmState(ArmState.PARK);
        setWristState(WristState.PARK);
    }

    // ---------------------------------------- Actuators --------------------------------------- //
    public void setArmState(ArmState state) {
        armState = state;
        armLeft.setPosition((double)arm_positions.get(state));
        armRight.setPosition((double)arm_positions.get(state));
    }

    public void setWristState(WristState state) {
        wristState = state;
        wrist.setPosition((double)wrist_positions.get(state)+0.05);
    }

    // ------------------------------------- State Getters -------------------------------------- //
    public ArmState getArmState() {
        return armState;
    }

    public WristState getWristState() {
        return wristState;
    }
}
