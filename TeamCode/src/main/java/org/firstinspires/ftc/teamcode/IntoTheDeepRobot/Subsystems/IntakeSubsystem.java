package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.hardware.ColorSensor;

import java.util.HashMap;

public class IntakeSubsystem extends SubsystemBase {
    private ServoImplEx wiper, raiseServo;
    private CRServoImplEx rightIntake, leftIntake;
    private ColorSensor colorSensor;
    private DigitalChannel limitSwitch, raiseLimitSwitch;
    private Telemetry telemetry;

    // ------------------------------------------ States ---------------------------------------- //
    // Intake Constant
    private static double intaking_power = 1.0;
    private static double brake_power = 0.4;

    // Raise States
    public enum RaiseState {
        RAISED,
        LOWERED,
        HANGING
    }

    private RaiseState raiseState;

    // Wiper States
    public enum WiperState {
        FULL_OPEN,
        SEMI_OPEN,
        CONTRACTED
    }

    private WiperState wiperState;

    private final HashMap<WiperState, Double> wiper_positions = new HashMap<WiperState, Double>() {{
        put(WiperState.FULL_OPEN, 0.0);
        put(WiperState.SEMI_OPEN, 0.1);
        put(WiperState.CONTRACTED, 0.45);
    }};

    private final HashMap<RaiseState, Double> raise_positions = new HashMap<RaiseState, Double>() {{
        put(RaiseState.RAISED, 0.5);
        put(RaiseState.LOWERED, 0.31);
        put(RaiseState.HANGING, 0.5);
    }};

    // Intake States
    public enum IntakeState {
        INTAKE,
        REVERSE,
        STOPPED
    }

    private IntakeState intakeState;

    // Color Sensor States
    public enum COLOR {
        NONE,
        RED,
        BLUE,
        YELLOW
    }

    private COLOR color;

    // ------------------------------------------------------------------------------------------ //
    public IntakeSubsystem(RobotMap robotMap) {
        this.telemetry = robotMap.getTelemetry();

        wiper = robotMap.getWiper();
        wiper_contract();

        raiseServo = robotMap.getIntakeRaiseServo();
        raise();

        leftIntake = robotMap.getLeftIntakeServo();
        rightIntake = robotMap.getRightIntakeServo();
        stop();

        colorSensor = robotMap.getColorSensor();
        color = COLOR.NONE;

        limitSwitch = robotMap.getSampleLimitSwitch();
        raiseLimitSwitch = robotMap.getRaiseLimitSwitch();
    }

    // ---------------------------------------- Actuators --------------------------------------- //
    // Raise
    public void raise() {
        raiseState = RaiseState.RAISED;
        raiseServo.setPosition((double)raise_positions.get(RaiseState.RAISED));
    }

    public void lower() {
        raiseState = RaiseState.LOWERED;
        raiseServo.setPosition((double)raise_positions.get(RaiseState.LOWERED));
    }

    public void hang() {
        raiseState = RaiseState.HANGING;
        raiseServo.setPosition((double)raise_positions.get(RaiseState.HANGING));
    }

    // Wiper
    public void wiper_full_open() {
        wiperState = WiperState.FULL_OPEN;
        wiper.setPosition((double)wiper_positions.get(WiperState.FULL_OPEN));
    }

    public void wiper_semi_open() {
        wiperState = WiperState.SEMI_OPEN;
        wiper.setPosition((double)wiper_positions.get(WiperState.SEMI_OPEN));
    }

    public void wiper_contract() {
        wiperState = WiperState.CONTRACTED;
        wiper.setPosition((double)wiper_positions.get(WiperState.CONTRACTED));
    }

    // Intake
    public void run() {
        intakeState = IntakeState.INTAKE;
        rightIntake.setPower(intaking_power);
        leftIntake.setPower(-intaking_power);
    }

    public void reverse() {
        intakeState = IntakeState.REVERSE;
        rightIntake.setPower(-intaking_power);
        leftIntake.setPower(intaking_power);
    }

    public void brake_reverse() {
        intakeState = IntakeState.INTAKE;
        rightIntake.setPower(-brake_power);
        leftIntake.setPower(brake_power);
    }

    public void stop() {
        intakeState = IntakeState.STOPPED;
        rightIntake.setPower(0);
        leftIntake.setPower(0);
    }

    // ---------------------------------------- Sensors ----------------------------------------- //
    private COLOR predict(double r, double g, double b) {
        if(b < 45 && r > 50 && g > 90) return COLOR.YELLOW;
        if(b > 45 && r < 35 && g < 35) return COLOR.BLUE;
        if(b < 35 && g < 35 && r > 35) return COLOR.RED;
        return COLOR.NONE;
    }

    // ----------------------------------------- Getters ---------------------------------------- //
    public RaiseState getRaiseState() {
        return raiseState;
    }
    public IntakeState getIntakeState() {
        return intakeState;
    }
    public WiperState getWiperState() {
        return wiperState;
    }

    public COLOR getSampleColor() {
        double[] colors = colorSensor.getNormalizedColors();

        telemetry.addData("Intake Red: ", colors[0]);
        telemetry.addData("Intake Green: ", colors[1]);
        telemetry.addData("Intake Blue: ", colors[2]);
        telemetry.addData("Intake Color Prediction: ", predict(colors[0], colors[1], colors[2]));

        return predict(colors[0], colors[1], colors[2]);
    }

    public boolean check_color(RobotEx.Alliance alliance, boolean for_basket) {
        if (!for_basket) return (getSampleColor() == (alliance == RobotEx.Alliance.RED ? COLOR.RED : COLOR.BLUE))
                ||
                getSampleColor() == COLOR.NONE;

        return (getSampleColor() == (alliance == RobotEx.Alliance.RED ? COLOR.RED : COLOR.BLUE))
                ||
                getSampleColor() == COLOR.YELLOW
                ||
                getSampleColor() == COLOR.NONE;
    }

    public boolean isSample() {
        telemetry.addData("Intake has Sample?: ", limitSwitch.getState());
        return limitSwitch.getState();
    }

    public ColorSensor getColorSensor() {
        return colorSensor;
    }
}
