package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.RobotMapInterface;
import org.inventors.ftc.robotbase.hardware.Battery;
import org.inventors.ftc.robotbase.hardware.ColorSensor;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;
import org.openftc.easyopencv.OpenCvWebcam;

public class RobotMap implements RobotMapInterface {
    public enum OpMode {
        AUTO,
        TELEOP
    }
    private OpMode opMode;

    private Telemetry telemetry;
    private GamepadExEx driverOp, toolOp;
    private MotorExEx frontLeft, frontRight, rearLeft, rearRight;

    private IMU imu;
    private OpenCvWebcam webcam;
    private Battery battery;

    //// ------------------------------------- Mechanisms ------------------------------------- ////
    // ------------------------------------------ Claw ------------------------------------------ //
    private ServoImplEx clawServo, clawRotServo;

    // ------------------------------------------ Arm ------------------------------------------- //
    private ServoImplEx armRightServo, armLeftServo, armWristServo;

    // ----------------------------------------- Intake ----------------------------------------- //
    private ServoImplEx intakeRaiseServoL, intakeRaiseServoR;
    private CRServoImplEx leftIntakeServo, rightIntakeServo;
    private ColorSensor colorSensor;
    private DigitalChannel sampleLimitSwitch, raiseLimitSwitch;

    // ----------------------------------------- Slider ----------------------------------------- //
    private MotorExEx sliderMotor, sliderFollow;

    // ----------------------------------------- Extendo ---------------------------------------- //
    private MotorExEx extendoMotor;

    // ---------------------------------------- Hanging ----------------------------------------- //
    private ServoImplEx hangingReleaseServo1, hangingReleaseServo2;

    // ----------------------------------------- Couplers --------------------------------------- //
    private ServoImplEx coupler_servo;

    // -------------------------------------- Distance Sensors ---------------------------------- //
    private AnalogInput rear_dist, left_dist, right_dist;

    public RobotMap(HardwareMap hm, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                    OpMode opMode) {
        this.opMode = opMode;
        if(opMode == OpMode.TELEOP) {
            frontLeft = new MotorExEx(hm, "front_left", Motor.GoBILDA.RPM_435);
            frontRight = new MotorExEx(hm, "front_right", Motor.GoBILDA.RPM_435);
            rearLeft = new MotorExEx(hm, "rear_left", Motor.GoBILDA.RPM_435);
            rearRight = new MotorExEx(hm, "rear_right", Motor.GoBILDA.RPM_435);
            frontLeft.setRunMode(Motor.RunMode.RawPower);
            frontRight.setRunMode(Motor.RunMode.RawPower);
            rearLeft.setRunMode(Motor.RunMode.RawPower);
            rearRight.setRunMode(Motor.RunMode.RawPower);

            imu = hm.get(IMU.class, "external_imu");
            IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
            );
            imu.initialize(imuParameters);

            driverOp = new GamepadExEx(gamepad1);
            toolOp = new GamepadExEx(gamepad2);
        }

        this.telemetry = telemetry;
        battery = new Battery(hm);

        //// ----------------------------------- Mechanisms ----------------------------------- ////
        // ---------------------------------------- Claw ---------------------------------------- //
        clawServo = hm.get(ServoImplEx.class, "claw");
        clawRotServo = hm.get(ServoImplEx.class, "claw_rot");

        // ---------------------------------------- Arm ----------------------------------------- //
        armLeftServo = hm.get(ServoImplEx.class, "arm_left");
        armRightServo = hm.get(ServoImplEx.class, "arm_right");
        armLeftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        armRightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        armWristServo = hm.get(ServoImplEx.class, "wrist_tilt");

        // --------------------------------------- Intake --------------------------------------- //
        intakeRaiseServoL = hm.get(ServoImplEx.class, "intake_raise_left");
        intakeRaiseServoR = hm.get(ServoImplEx.class, "intake_raise_right");

        leftIntakeServo = hm.get(CRServoImplEx.class, "intake_left_wheel");
        rightIntakeServo = hm.get(CRServoImplEx.class, "intake_right_wheel");

        colorSensor = new ColorSensor(hm, "intake_color");
        colorSensor.setGain(100);

        sampleLimitSwitch = hm.get(DigitalChannel.class, "intake_sample_switch");
//        raiseLimitSwitch = hm.get(DigitalChannel.class, "raise_switch");

        // ---------------------------------------- Slider -------------------------------------- //
        sliderMotor = new MotorExEx(hm, "slider", 383.6, 435);
        sliderFollow = new MotorExEx(hm, "slider2", 383.6, 435);

        // ---------------------------------------- Extendo ------------------------------------- //
        extendoMotor = new MotorExEx(hm, "extendo", Motor.GoBILDA.RPM_435);

        // ---------------------------------------- Hanging ------------------------------------- //
        hangingReleaseServo1 = hm.get(ServoImplEx.class, "release1");
        hangingReleaseServo2 = hm.get(ServoImplEx.class, "release2");

        // --------------------------------------- Couplers ------------------------------------- //
        coupler_servo = hm.get(ServoImplEx.class, "coupler");

        // ------------------------------------ Distance Sensors -------------------------------- //
        rear_dist = hm.get(AnalogInput.class, "rear_dist");
        left_dist = hm.get(AnalogInput.class, "left_dist");
        right_dist = hm.get(AnalogInput.class, "right_dist");
    }

    public OpMode getOpMode() {
        return opMode;
    }

    // ------------------------------------ Drivetrain Motors ----------------------------------- //
    @Override
    public MotorExEx getFrontLeftMotor() {
        return frontLeft;
    }

    @Override
    public MotorExEx getFrontRightMotor() {
        return frontRight;
    }

    @Override
    public MotorExEx getRearLeftMotor() {
        return rearLeft;
    }

    @Override
    public MotorExEx getRearRightMotor() {
        return rearRight;
    }

    // ----------------------------------------- Telemetry -------------------------------------- //
    @Override
    public Telemetry getTelemetry() {
        return telemetry;
    }

    // ------------------------------------------ Sensors --------------------------------------- //
    @Override
    public OpenCvWebcam getCamera() {
        return webcam;
    }

    @Override
    public IMU getIMU() {
        return imu;
    }

    // ------------------------------------------ Gamepads -------------------------------------- //
    @Override
    public GamepadExEx getDriverOp() {
        return driverOp;
    }

    @Override
    public GamepadExEx getToolOp() {
        return toolOp;
    }

    // ------------------------------------------- Battery -------------------------------------- //
    @Override
    public Battery getBattery() {
        return battery;
    }

    //// --------------------------------------- Mechanisms ----------------------------------- ////
    // ------------------------------------------- Claw ----------------------------------------- //
    public ServoImplEx getClawServo() {
        return clawServo;
    }

    public ServoImplEx getClawRotServo() {
        return clawRotServo;
    }

    // ------------------------------------------- Arm ------------------------------------------ //
    public ServoImplEx getArmRightServo() {
        return armRightServo;
    }

    public ServoImplEx getArmLeftServo() {
        return armLeftServo;
    }

    public ServoImplEx getArmWristServo() {
        return armWristServo;
    }

    // ----------------------------------------- Intake ----------------------------------------- //
    public ServoImplEx getIntakeRaiseServoL() {
        return intakeRaiseServoL;
    }

    public ServoImplEx getIntakeRaiseServoR() {
        return intakeRaiseServoR;
    }

    public CRServoImplEx getLeftIntakeServo() {
        return leftIntakeServo;
    }

    public CRServoImplEx getRightIntakeServo() {
        return rightIntakeServo;
    }

    public ColorSensor getColorSensor() {
        return colorSensor;
    }

    public DigitalChannel getSampleLimitSwitch() {
        return sampleLimitSwitch;
    }

    public DigitalChannel getRaiseLimitSwitch() {
        return raiseLimitSwitch;
    }

    // ----------------------------------------- Slider ----------------------------------------- //
    public MotorExEx getSliderMotor() {
        return sliderMotor;
    }

    public MotorExEx getSliderFollow() {
        return sliderFollow;
    }

    // ----------------------------------------- Extendo ---------------------------------------- //
    public MotorExEx getExtendoMotor() {
        return extendoMotor;
    }

    // ---------------------------------------- Hanging ----------------------------------------- //
    public ServoImplEx getHangingReleaseServo1() {
        return hangingReleaseServo1;
    }

    public ServoImplEx getHangingReleaseServo2() {
        return hangingReleaseServo2;
    }

    // ----------------------------------------- Couplers --------------------------------------- //
    public ServoImplEx getCouplerServo() {
        return coupler_servo;
    }

    // -------------------------------------- Distance Sensors ---------------------------------- //
    public AnalogInput getRearDist() {
        return rear_dist;
    }

    public AnalogInput getLeftDist() {
        return left_dist;
    }

    public AnalogInput getRightDist() {
        return right_dist;
    }
}
