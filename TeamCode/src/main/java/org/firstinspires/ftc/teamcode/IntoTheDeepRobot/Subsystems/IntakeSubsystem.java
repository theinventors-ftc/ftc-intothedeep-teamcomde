package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;

public class IntakeSubsystem extends SubsystemBase {
    private ServoImplEx raiseServoL, raiseServoR;
    private CRServoImplEx rightIntake, leftIntake;
    private NormalizedColorSensor colorSensor;
    private DigitalChannel limitSwitch;
    private Telemetry telemetry;

    // ------------------------------------------ States ---------------------------------------- //
    // Intake Contstant
    private static double intaking_power = 0.9;
    private static double brake_power = 0.4;

    // Raise States
    public enum RaiseState {
        RAISED,
        LOWERED
    }

    private RaiseState state;

    private HashMap<RaiseState, Double> raise_positionsL = new HashMap<RaiseState, Double>() {{
        put(RaiseState.RAISED, 0.2);
        put(RaiseState.LOWERED, 0.5);
    }};

    private HashMap<RaiseState, Double> raise_positionsR = new HashMap<RaiseState, Double>() {{
        put(RaiseState.RAISED, 0.8);
        put(RaiseState.LOWERED, 0.5);
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
    public IntakeSubsystem(HardwareMap hm, Telemetry telemetry) {
        this.telemetry = telemetry;

        raiseServoL = hm.get(ServoImplEx.class, "intake_raise_left");
        raiseServoR = hm.get(ServoImplEx.class, "intake_raise_right");
        raise();

        leftIntake = hm.get(CRServoImplEx.class, "intake_left_wheel");
        rightIntake = hm.get(CRServoImplEx.class, "intake_right_wheel");
        stop();

        colorSensor = hm.get(NormalizedColorSensor.class, "color");
        colorSensor.setGain(100);
        color = COLOR.NONE;

        limitSwitch = hm.get(DigitalChannel.class, "switch");
    }

    // ---------------------------------------- Actuators --------------------------------------- //
    // Raise
    public void raise() {
        state = RaiseState.RAISED;
        raiseServoL.setPosition((double)raise_positionsL.get(RaiseState.RAISED));
        raiseServoR.setPosition((double)raise_positionsR.get(RaiseState.RAISED));
    }

    public void lower() {
        state = RaiseState.LOWERED;
        raiseServoL.setPosition((double)raise_positionsL.get(RaiseState.LOWERED));
        raiseServoR.setPosition((double)raise_positionsR.get(RaiseState.LOWERED));
    }

    // Intake
    public void run() {
        intakeState = IntakeState.INTAKE;
        rightIntake.setPower(-intaking_power);
        leftIntake.setPower(intaking_power);
    }

    public void reverse() {
        intakeState = IntakeState.REVERSE;
        rightIntake.setPower(intaking_power);
        leftIntake.setPower(-intaking_power);
    }

    public void brake_reverse() {
        intakeState = IntakeState.INTAKE;
        rightIntake.setPower(brake_power);
        leftIntake.setPower(-brake_power);
    }

    public void stop() {
        intakeState = IntakeState.STOPPED;
        rightIntake.setPower(0);
        leftIntake.setPower(0);
    }

    // ---------------------------------------- Sensors ----------------------------------------- //
    private COLOR predict(double r, double g, double b) {
        if(b < 60 && r > 90 && g > 90) return COLOR.YELLOW;
        if(b > 70 && r < 50 && g < 60) return COLOR.BLUE;
        if(b < 40 && g < 65 && r > 65) return COLOR.RED;
        return COLOR.NONE;
    }

    // ----------------------------------------- Getters ---------------------------------------- //
    public RaiseState getState() {
        return state;
    }

    public COLOR getSampleColor() {
        NormalizedRGBA raw_colors = colorSensor.getNormalizedColors();
        double[] colors = {Math.floor(raw_colors.red*100), Math.floor(raw_colors.green*100), Math.floor(raw_colors.blue*100), Math.floor(raw_colors.alpha*100)};

        telemetry.addData("Red: ", colors[0]);
        telemetry.addData("Green: ", colors[1]);
        telemetry.addData("Blue: ", colors[2]);
        telemetry.addData("Color Prediction: ", predict(colors[0], colors[1], colors[2]));

        return predict(colors[0], colors[1], colors[2]);
    }

    public boolean isSample() {
        telemetry.addData("Is Sample: ", limitSwitch.getState());
        return limitSwitch.getState();
    }
}