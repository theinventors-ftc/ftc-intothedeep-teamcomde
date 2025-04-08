package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
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
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;
import org.inventors.ftc.robotbase.RobotMapInterface;
import org.inventors.ftc.robotbase.hardware.Battery;
import org.inventors.ftc.robotbase.hardware.ColorSensor;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;
import org.jetbrains.annotations.NotNull;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.PipelineRecordingParameters;

public class RobotMap implements RobotMapInterface {
    public enum OpMode {
        AUTO,
        TELEOP
    }
    private OpMode opMode;

    private HardwareMap hardwareMap;

    private Telemetry telemetry;
    private GamepadExEx driverOp, toolOp;
    private MotorExEx frontLeft, frontRight, rearLeft, rearRight;

    private IMU imu;
    private OpenCvWebcam frontCamera;
    private Battery battery;

    //// ------------------------------------- Mechanisms ------------------------------------- ////
    // ------------------------------------------ Claw ------------------------------------------ //
    private ServoImplEx clawServo, clawRotServo;

    // ------------------------------------------ Arm ------------------------------------------- //
    private ServoImplEx armRightServo, armLeftServo, armWristServo;

    // ----------------------------------------- Intake ----------------------------------------- //
    private ServoImplEx wiper, intakeRaiseServoR;
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

    private Limelight3A limelight;

    public RobotMap(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2,
                    OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        if(opMode == OpMode.TELEOP) {
            driverOp = new GamepadExEx(gamepad1);
            toolOp = new GamepadExEx(gamepad2);
            // ------------------------------------- Drivetrain ------------------------------------- //
            frontLeft = new MotorExEx(hardwareMap, "front_left", Motor.GoBILDA.RPM_435);
            frontRight = new MotorExEx(hardwareMap, "front_right", Motor.GoBILDA.RPM_435);
            rearLeft = new MotorExEx(hardwareMap, "rear_left", Motor.GoBILDA.RPM_435);
            rearRight = new MotorExEx(hardwareMap, "rear_right", Motor.GoBILDA.RPM_435);
            frontLeft.setRunMode(Motor.RunMode.RawPower);
            frontRight.setRunMode(Motor.RunMode.RawPower);
            rearLeft.setRunMode(Motor.RunMode.RawPower);
            rearRight.setRunMode(Motor.RunMode.RawPower);

            imu = hardwareMap.get(IMU.class, "external_imu");
            IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                    RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                )
            );
            imu.initialize(imuParameters);
        }

        // ---------------------------------------- Util ---------------------------------------- //
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.telemetry = telemetry;
        battery = new Battery(hardwareMap);

        // Camera
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//        telemetry.setMsTransmissionInterval(11);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources()
//                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        rearCamera =  OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,
//                "rear_camera"), cameraMonitorViewId);
//
//        FtcDashboard.getInstance().startCameraStream(rearCamera, 0);
//        rearCamera.setMillisecondsPermissionTimeout(2500);
//        rearCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                rearCamera.startStreaming(
//                        320, // 320
//                        180, //180
//                        OpenCvCameraRotation.UPSIDE_DOWN,
//                        OpenCvWebcam.StreamFormat.MJPEG
//                );
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });


        //// ----------------------------------- Mechanisms ----------------------------------- ////
        // ---------------------------------------- Claw ---------------------------------------- //
        clawServo = hardwareMap.get(ServoImplEx.class, "claw");
        clawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawRotServo = hardwareMap.get(ServoImplEx.class, "claw_rot");

        // ---------------------------------------- Arm ----------------------------------------- //
        armLeftServo = hardwareMap.get(ServoImplEx.class, "arm_left");
        armRightServo = hardwareMap.get(ServoImplEx.class, "arm_right");
        armLeftServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        armRightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        armWristServo = hardwareMap.get(ServoImplEx.class, "wrist_tilt");
        armWristServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // --------------------------------------- Intake --------------------------------------- //
        wiper = hardwareMap.get(ServoImplEx.class, "wiper");
        intakeRaiseServoR = hardwareMap.get(ServoImplEx.class, "intake_raise_right");

        leftIntakeServo = hardwareMap.get(CRServoImplEx.class, "intake_left_wheel");
        rightIntakeServo = hardwareMap.get(CRServoImplEx.class, "intake_right_wheel");

        colorSensor = new ColorSensor(hardwareMap, "intake_color");
        colorSensor.setGain(100);

        sampleLimitSwitch = hardwareMap.get(DigitalChannel.class, "intake_sample_switch");
//        raiseLimitSwitch = hm.get(DigitalChannel.class, "raise_switch");

        // ---------------------------------------- Slider -------------------------------------- //
        sliderMotor = new MotorExEx(hardwareMap, "slider", 383.6, 435);
        sliderFollow = new MotorExEx(hardwareMap, "slider2", 383.6, 435);

        // ---------------------------------------- Extendo ------------------------------------- //
        extendoMotor = new MotorExEx(hardwareMap, "extendo", Motor.GoBILDA.RPM_435);

        // ---------------------------------------- Hanging ------------------------------------- //
        hangingReleaseServo1 = hardwareMap.get(ServoImplEx.class, "release1");
        hangingReleaseServo2 = hardwareMap.get(ServoImplEx.class, "release2");

        // --------------------------------------- Couplers ------------------------------------- //
        coupler_servo = hardwareMap.get(ServoImplEx.class, "coupler");

        // ------------------------------------ Distance Sensors -------------------------------- //
        rear_dist = hardwareMap.get(AnalogInput.class, "rear_dist");
        left_dist = hardwareMap.get(AnalogInput.class, "left_dist");
        right_dist = hardwareMap.get(AnalogInput.class, "right_dist");
    }

    public OpMode getOpMode() {
        return opMode;
    }

    // ------------------------------------ Drivetrain Motors ----------------------------------- //
    @NotNull
    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

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
        return new OpenCvWebcam() {
            @Override
            public void setMillisecondsPermissionTimeout(int ms) {

            }

            @Override
            public void startStreaming(int width, int height, OpenCvCameraRotation rotation, StreamFormat streamFormat) {

            }

            @Override
            public ExposureControl getExposureControl() {
                return null;
            }

            @Override
            public FocusControl getFocusControl() {
                return null;
            }

            @Override
            public PtzControl getPtzControl() {
                return null;
            }

            @Override
            public GainControl getGainControl() {
                return null;
            }

            @Override
            public WhiteBalanceControl getWhiteBalanceControl() {
                return null;
            }

            @Override
            public <T extends CameraControl> T getControl(Class<T> controlType) {
                return null;
            }

            @Override
            public CameraCalibrationIdentity getCalibrationIdentity() {
                return null;
            }

            @Override
            public int openCameraDevice() {
                return 0;
            }

            @Override
            public void openCameraDeviceAsync(AsyncCameraOpenListener cameraOpenListener) {

            }

            @Override
            public void closeCameraDevice() {

            }

            @Override
            public void closeCameraDeviceAsync(AsyncCameraCloseListener cameraCloseListener) {

            }

            @Override
            public void showFpsMeterOnViewport(boolean show) {

            }

            @Override
            public void pauseViewport() {

            }

            @Override
            public void resumeViewport() {

            }

            @Override
            public void setViewportRenderingPolicy(ViewportRenderingPolicy policy) {

            }

            @Override
            public void setViewportRenderer(ViewportRenderer renderer) {

            }

            @Override
            public void startStreaming(int width, int height) {

            }

            @Override
            public void startStreaming(int width, int height, OpenCvCameraRotation rotation) {

            }

            @Override
            public void stopStreaming() {

            }

            @Override
            public void setPipeline(OpenCvPipeline pipeline) {

            }

            @Override
            public int getFrameCount() {
                return 0;
            }

            @Override
            public float getFps() {
                return 0;
            }

            @Override
            public int getPipelineTimeMs() {
                return 0;
            }

            @Override
            public int getOverheadTimeMs() {
                return 0;
            }

            @Override
            public int getTotalFrameTimeMs() {
                return 0;
            }

            @Override
            public int getCurrentPipelineMaxFps() {
                return 0;
            }

            @Override
            public void startRecordingPipeline(PipelineRecordingParameters parameters) {

            }

            @Override
            public void stopRecordingPipeline() {

            }

            @Override
            public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {

            }
        };
    }

    public Limelight3A getRearLimelight() {
        return limelight;
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
    public ServoImplEx getWiper () {
        return wiper;
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
