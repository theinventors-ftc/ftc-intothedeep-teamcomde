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
        HIGH,
        PERP,
        SPECIMENT_INTAKE,
        INTAKE_B_ABOVE,
        INTAKE_B,
        BASKET_OUTTAKE,
        SPECIMENT_OUTTAKE_HIGH,
        SPECIMENT_OUTTAKE_LOW,
        HUMAN_PLAYER
    }
    public enum WristState {
        INTAKE,
        PARK,
        HIGH,
        PERP,
        SPECIMENT_INTAKE,
        INTAKE_B_ABOVE,
        INTAKE_B,
        BASKET_OUTTAKE,
        SPECIMENT_OUTTAKE_HIGH,
        SPECIMENT_OUTTAKE_LOW,
        HUMAN_PLAYER
    }
    private ArmState armState;
    private WristState wristState;

    private final HashMap<ArmState, Double> arm_positions = new HashMap<ArmState, Double>() {{
        put(ArmState.INTAKE, 0.075);
        put(ArmState.PARK, 0.17);
        put(ArmState.HIGH, 0.45);
        put(ArmState.PERP, 0.75);
        put(ArmState.SPECIMENT_INTAKE, 0.77);
        put(ArmState.INTAKE_B_ABOVE, 0.88);
        put(ArmState.INTAKE_B, 0.91);
        put(ArmState.BASKET_OUTTAKE, 0.46);
        put(ArmState.SPECIMENT_OUTTAKE_HIGH, 0.71);
        put(ArmState.SPECIMENT_OUTTAKE_LOW, 0.86);
        put(ArmState.HUMAN_PLAYER, 0.88);
    }};

    private final HashMap<WristState, Double> wrist_positions = new HashMap<WristState, Double>() {{
        put(WristState.INTAKE, 0.2);
        put(WristState.PARK, 0.09);
        put(WristState.HIGH, 0.52);
        put(WristState.PERP, 0.48);
        put(WristState.SPECIMENT_INTAKE, 0.47);
        put(WristState.INTAKE_B_ABOVE, 0.70);
        put(WristState.INTAKE_B, 0.67);
        put(WristState.BASKET_OUTTAKE, 0.78);
        put(WristState.SPECIMENT_OUTTAKE_HIGH, 0.17);
        put(WristState.SPECIMENT_OUTTAKE_LOW, 0.0);
        put(WristState.HUMAN_PLAYER, 0.69);
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
        wrist.setPosition((double)wrist_positions.get(state));
    }

    // ------------------------------------- State Getters -------------------------------------- //
    public ArmState getArmState() {
        return armState;
    }

    public WristState getWristState() {
        return wristState;
    }
}
