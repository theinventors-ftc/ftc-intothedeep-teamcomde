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
        PERP,
        INTAKE_B_ABOVE,
        INTAKE_B,
        BASKET_OUTTAKE,
        SPECIMENT_OUTTAKE,
        SPECIMENT_LOW_OUTTAKE
    }
    public enum WristState {
        INTAKE,
        PARK,
        HIGH,
        PERP,
        INTAKE_B_ABOVE,
        INTAKE_B,
        BASKET_OUTTAKE,
        SPECIMENT_OUTTAKE,
        SPECIMENT_LOW_OUTTAKE
    }
    private ArmState armState;
    private WristState wristState;

    private final HashMap<ArmState, Double> arm_positionsL = new HashMap<ArmState, Double>() {{
        put(ArmState.INTAKE, 0.075);
        put(ArmState.PARK, 0.17);
        put(ArmState.HIGH, 0.45);
        put(ArmState.PERP, 0.75);
        put(ArmState.INTAKE_B_ABOVE, 0.88);
        put(ArmState.INTAKE_B, 0.91);
        put(ArmState.BASKET_OUTTAKE, 0.46);
        put(ArmState.SPECIMENT_OUTTAKE, 0.7);
        put(ArmState.SPECIMENT_LOW_OUTTAKE, 0.84);
    }};

//    private final HashMap<ArmState, Double> arm_positionsR = new HashMap<ArmState, Double>() {{
//        put(ArmState.INTAKE, 0.075);
//        put(ArmState.PARK, 0.17);
//        put(ArmState.HIGH, 0.45);
//        put(ArmState.PERP, 0.75);
//        put(ArmState.INTAKE_B_ABOVE, 0.88);
//        put(ArmState.INTAKE_B, 0.91);
//        put(ArmState.BASKET_OUTTAKE, 0.46);
//        put(ArmState.SPECIMENT_OUTTAKE, 0.7);
//        put(ArmState.SPECIMENT_LOW_OUTTAKE, 0.84);
//    }};

    private final HashMap<WristState, Double> wrist_positions = new HashMap<WristState, Double>() {{
        put(WristState.INTAKE, 0.23);
        put(WristState.PARK, 0.09);
        put(WristState.HIGH, 0.52);
        put(WristState.PERP, 0.48);
        put(WristState.INTAKE_B_ABOVE, 0.70);
        put(WristState.INTAKE_B, 0.67);
        put(WristState.BASKET_OUTTAKE, 0.78);
        put(WristState.SPECIMENT_OUTTAKE, 0.16);
        put(WristState.SPECIMENT_LOW_OUTTAKE, 0.0);
    }};

    public ArmSubsystem(RobotMap robotMap) {
        armLeft = robotMap.getArmLeftServo();
        armRight = robotMap.getArmRightServo();
        wrist = robotMap.getArmWristServo();

        arm_goPark();
        wrist_goPark();
    }

    // ---------------------------------------- Actuators --------------------------------------- //

    public void setArmState(ArmState state) {
        armState = state;
        armLeft.setPosition((double)arm_positionsL.get(state));
        armRight.setPosition((double)arm_positionsL.get(state));
    }

    public void arm_goIntake() {
        setArmState(ArmState.INTAKE);
    }

    public void arm_goPark() {
        setArmState(ArmState.PARK);
    }

    public void arm_goHigh() {
        setArmState(ArmState.HIGH);
    }

    public void arm_goPerp() {
        setArmState(ArmState.PERP);
    }

    public void arm_goIntakeBAbove() {
        setArmState(ArmState.INTAKE_B_ABOVE);
    }

    public void arm_goIntakeB() {
        setArmState(ArmState.INTAKE_B);
    }

    public void arm_goBasketOuttake() {
        setArmState(ArmState.BASKET_OUTTAKE);
    }

    public void arm_goSpecimentOuttake() {
        setArmState(ArmState.SPECIMENT_OUTTAKE);
    }

    public void arm_goSpecimentOuttakeLow() {
        setArmState(ArmState.SPECIMENT_LOW_OUTTAKE);
    }

    public void setWristState(WristState state) {
        wristState = state;
        wrist.setPosition((double)wrist_positions.get(state));
    }

    public void wrist_goIntake() {
        setWristState(WristState.INTAKE);
    }

    public void wrist_goPark() {
        setWristState(WristState.PARK);
    }

    public void wrist_goHigh() {
        setWristState(WristState.HIGH);
    }

    public void wrist_goPerp() {
        setWristState(WristState.PERP);
    }

    public void wrist_goIntakeBAbove() {
        setWristState(WristState.INTAKE_B_ABOVE);
    }

    public void wrist_goIntakeB() {
        setWristState(WristState.INTAKE_B);
    }

    public void wrist_goBasketOuttake() {
        setWristState(WristState.BASKET_OUTTAKE);
    }

    public void wrist_goSpecimentOuttake() {
        setWristState(WristState.SPECIMENT_OUTTAKE);
    }

    public void wrist_goSpecimentOuttakeLow() {
        setWristState(WristState.SPECIMENT_LOW_OUTTAKE);
    }

    // ------------------------------------- State Getters -------------------------------------- //
    public ArmState getArmState() {
        return armState;
    }

    public WristState getWristState() {
        return wristState;
    }
}
