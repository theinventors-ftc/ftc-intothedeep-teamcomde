package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.RobotMapInterface;
import org.inventors.ftc.robotbase.hardware.Battery;
import org.inventors.ftc.robotbase.hardware.ColorSensor;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;
import org.openftc.easyopencv.OpenCvWebcam;

public class RobotMap implements RobotMapInterface {
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
    private DigitalChannel limitSwitch;

    // ----------------------------------------- Slider ----------------------------------------- //

    // ----------------------------------------- Extendo ---------------------------------------- //

    // ----------------------------------------- Couplers --------------------------------------- //
    private ServoImplEx coupler_servo;

    public RobotMap(HardwareMap hm, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        frontLeft = new MotorExEx(hm, "front_left", Motor.GoBILDA.RPM_435);
        frontRight = new MotorExEx(hm, "front_right", Motor.GoBILDA.RPM_435);
        rearLeft = new MotorExEx(hm, "rear_left", Motor.GoBILDA.RPM_435);
        rearRight = new MotorExEx(hm, "rear_right", Motor.GoBILDA.RPM_435);
        this.telemetry = telemetry;

//        int cameraMonitorViewId = hm.appContext.getResources()
//                .getIdentifier("cameraMonitorViewId", "id", hm.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hm.get(WebcamName.class,
//                "webcam"), cameraMonitorViewId);
        imu = hm.get(IMU.class, "external_imu");

        driverOp = new GamepadExEx(gamepad1);
        toolOp = new GamepadExEx(gamepad2);

        battery = new Battery(hm);

        //// ----------------------------------- Mechanisms ----------------------------------- ////
        // ---------------------------------------- Claw ---------------------------------------- //
        clawServo = hm.get(ServoImplEx.class, "claw");
        clawRotServo = hm.get(ServoImplEx.class, "claw_rot");

        // ---------------------------------------- Arm ----------------------------------------- //
        armLeftServo = hm.get(ServoImplEx.class, "arm_left");
        armRightServo = hm.get(ServoImplEx.class, "arm_right");
        armWristServo = hm.get(ServoImplEx.class, "wrist");

        // --------------------------------------- Intake --------------------------------------- //
        intakeRaiseServoL = hm.get(ServoImplEx.class, "intake_raise_left");
        intakeRaiseServoR = hm.get(ServoImplEx.class, "intake_raise_right");

        leftIntakeServo = hm.get(CRServoImplEx.class, "intake_left_wheel");
        rightIntakeServo = hm.get(CRServoImplEx.class, "intake_right_wheel");

        colorSensor = new ColorSensor(hm, "color");
        colorSensor.setGain(100);

        limitSwitch = hm.get(DigitalChannel.class, "switch");

        // --------------------------------------- Couplers ------------------------------------- //
        coupler_servo = hm.get(ServoImplEx.class, "coupler");
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

    public DigitalChannel getLimitSwitch() {
        return limitSwitch;
    }

    // ----------------------------------------- Couplers --------------------------------------- //
    public ServoImplEx getCouplerServo() {
        return coupler_servo;
    }
}
