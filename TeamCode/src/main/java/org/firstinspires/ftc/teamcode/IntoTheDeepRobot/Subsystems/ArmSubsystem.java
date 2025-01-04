package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.HashMap;

public class ArmSubsystem extends SubsystemBase {
    private final ServoImplEx armLeft, armRight, wrist;

    // ------------------------------------------ States ---------------------------------------- //
    public enum ArmState {
        INTAKE,
        PARK,
        HIGH,
        PERP
    }
    public enum WristState {
        INTAKE,
        PARK,
        HIGH,
        PERP
    }
    private ArmState armState;
    private WristState wristState;

    HashMap<ArmState, Double> arm_positionsL = new HashMap<ArmState, Double>() {{
        put(ArmState.INTAKE, 0.2);
        put(ArmState.PARK, 0.5);
        put(ArmState.HIGH, 0.8);
        put(ArmState.PERP, 0.9);
    }};

    HashMap<ArmState, Double> arm_positionsR = new HashMap<ArmState, Double>() {{
        put(ArmState.INTAKE, 0.8);
        put(ArmState.PARK, 0.5);
        put(ArmState.HIGH, 0.2);
        put(ArmState.PERP, 0.1);
    }};

    HashMap<WristState, Double> wrist_positions = new HashMap<WristState, Double>() {{
        put(WristState.INTAKE, 0.2);
        put(WristState.PARK, 0.2);
        put(WristState.HIGH, 0.7);
        put(WristState.PERP, 0.7);
    }};

    public ArmSubsystem(RobotMap robotMap) {
        armLeft = robotMap.getArmLeftServo();
        armRight = robotMap.getArmRightServo();
        wrist = robotMap.getArmWristServo();

        arm_goPark();
        wrist_goPark();
    }

    // ---------------------------------------- Actuators --------------------------------------- //

    void setArmState(ArmState state) {
        armState = state;
        armLeft.setPosition((double)arm_positionsL.get(state));
        armRight.setPosition(1.0-(double)arm_positionsR.get(state));
    }

    void arm_goIntake() {
        setArmState(ArmState.INTAKE);
    }

    void arm_goPark() {
        setArmState(ArmState.PARK);
    }

    void arm_goHigh() {
        setArmState(ArmState.HIGH);
    }

    void arm_goPerp() {
        setArmState(ArmState.PERP);
    }

    void setWristState(WristState state) {
        wristState = state;
        wrist.setPosition((double)wrist_positions.get(state));
    }

    void wrist_goIntake() {
        setWristState(WristState.INTAKE);
    }

    void wrist_goPark() {
        setWristState(WristState.PARK);
    }

    void wrist_goHigh() {
        setWristState(WristState.HIGH);
    }

    void wrist_goPerp() {
        setWristState(WristState.PERP);
    }

    // ------------------------------------- State Getters -------------------------------------- //
    public ArmState getArmState() {
        return armState;
    }

    public WristState getWristState() {
        return wristState;
    }
}
