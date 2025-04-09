package org.firstinspires.ftc.teamcode.Tests;

import android.net.TransportInfo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeV2Subsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.SampleDetectionSubsystem;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;

@Config
@TeleOp(name = "ChickenIntakeServoPosTest", group = "Tests")
public class ChickenIntakeServoPosTest extends CommandOpMode {
    private GamepadExEx toolOp;
    private IntakeV2Subsystem intakeV2Subsystem;
    private SampleDetectionSubsystem sampleDetectionSubsystem;
//    public static boolean raise_enabled = false, arm_rot_enabled = false, arm_lift_enabled = false, claw_rot_enabled = false;
//    public static double raiseServoPos = 0.5, armRotPos = 0.5, armLiftPos = 0.5, clawRotPos = 0.5, raise_off=0.0;

//    public static IntakeV2Subsystem.RaiseState raiseState = IntakeV2Subsystem.RaiseState.INTAKE;
//    public static IntakeV2Subsystem.ArmRotState armRotState = IntakeV2Subsystem.ArmRotState.INTAKE;
//    public static IntakeV2Subsystem.ArmLiftState armLiftState = IntakeV2Subsystem.ArmLiftState.INTAKE;
//    public static IntakeV2Subsystem.ClawRotState clawRotState = IntakeV2Subsystem.ClawRotState.INTAKE;
//    public static IntakeV2Subsystem.ClawState clawState = IntakeV2Subsystem.ClawState.RELEASED;

    @Override
    public void initialize() {
        telemetry.setMsTransmissionInterval(11);
        intakeV2Subsystem = new IntakeV2Subsystem(hardwareMap, telemetry);
        toolOp = new GamepadExEx(gamepad2);
        sampleDetectionSubsystem = new SampleDetectionSubsystem(hardwareMap, telemetry);
        sampleDetectionSubsystem.enable();


        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(() -> intakeV2Subsystem.setRaiseState(IntakeV2Subsystem.RaiseState.PASSTHROUGH)),
                        new WaitCommand(1150),
                        new InstantCommand(() -> intakeV2Subsystem.setArmLiftState(IntakeV2Subsystem.ArmLiftState.PASSTHROUGH)),
                        new WaitCommand(200),
                        new InstantCommand(() -> intakeV2Subsystem.setArmRotState(IntakeV2Subsystem.ArmRotState.PASSTHROUGH)),
                        new InstantCommand(() -> intakeV2Subsystem.setClawRotState(IntakeV2Subsystem.ClawRotState.PASSTHROUGH))
                ),
                new SequentialCommandGroup(
                        new InstantCommand(() -> intakeV2Subsystem.setArmRotState(IntakeV2Subsystem.ArmRotState.INTAKE)),
                        new InstantCommand(() -> intakeV2Subsystem.setClawRotState(IntakeV2Subsystem.ClawRotState.INTAKE)),
                        new WaitCommand(50),
                        new InstantCommand(() -> intakeV2Subsystem.setRaiseState(IntakeV2Subsystem.RaiseState.INTAKE)),
                        new WaitCommand(750),
                        new InstantCommand(() -> intakeV2Subsystem.setArmLiftState(IntakeV2Subsystem.ArmLiftState.INTAKE_AIM))
                ),
                () -> intakeV2Subsystem.getRaiseState() == IntakeV2Subsystem.RaiseState.INTAKE
        ));

        new Trigger(() -> (toolOp.getGamepadButton(GamepadKeys.Button.B).get() && intakeV2Subsystem.getRaiseState() == IntakeV2Subsystem.RaiseState.INTAKE))
                .whenActive(new InstantCommand(() -> intakeV2Subsystem.setArmLiftState(IntakeV2Subsystem.ArmLiftState.INTAKE)))
                .whenInactive(new InstantCommand(() -> intakeV2Subsystem.setArmLiftState(IntakeV2Subsystem.ArmLiftState.INTAKE_AIM)));
    }

    @Override
    public void run() {
        super.run();

        if (Math.abs(toolOp.getRightX()) > 0.05 && intakeV2Subsystem.getRaiseState() == IntakeV2Subsystem.RaiseState.INTAKE) {
            intakeV2Subsystem.setArmRotState(IntakeV2Subsystem.ArmRotState.SAMPLE_FOLLOW);
            intakeV2Subsystem.setArmRotation(Range.scale(toolOp.getRightX(), -1, 1, 0.0, 0.185*2));
        }

        if (Math.abs(toolOp.getLeftX()) > 0.05 && intakeV2Subsystem.getRaiseState() == IntakeV2Subsystem.RaiseState.INTAKE) {
            intakeV2Subsystem.setClawRotState(IntakeV2Subsystem.ClawRotState.SAMPLE_FOLLOW);
//            intakeV2Subsystem.setClawRotation(Range.scale(toolOp.getLeftX(), -1, 1, 0.0, 0.43*2));
            intakeV2Subsystem.setClawRotation(Range.scale(sampleDetectionSubsystem.getAngle(), 0, 360, 0.0, 0.43));
        }

        telemetry.addData("Raise State", intakeV2Subsystem.getRaiseState());
        telemetry.addData("Arm Rot State", intakeV2Subsystem.getArmRotState());
        telemetry.addData("Arm Lift State", intakeV2Subsystem.getArmLiftState());
        telemetry.addData("Sample Rot", sampleDetectionSubsystem.getAngle());
        telemetry.update();
    }

//    @Override
//    public void runOpMode() throws InterruptedException {
//        intakeV2Subsystem = new IntakeV2Subsystem(hardwareMap, telemetry);
//
//        waitForStart();
//
//        while (opModeIsActive() && !isStopRequested()) {
////            intakeV2Subsystem.setRaiseState(IntakeV2Subsystem.RaiseState.values()[raiseState]);
////            intakeV2Subsystem.setArmRotState(IntakeV2Subsystem.ArmRotState.values()[armRotState]);
////            intakeV2Subsystem.setArmLiftState(IntakeV2Subsystem.ArmLiftState.values()[armLiftState]);
////            intakeV2Subsystem.setClawRotState(IntakeV2Subsystem.ClawRotState.values()[clawRotState]);
////            intakeV2Subsystem.setClawState(IntakeV2Subsystem.ClawState.values()[clawState]);
//            intakeV2Subsystem.setRaiseState(raiseState);
//            intakeV2Subsystem.setArmRotState(armRotState);
//            intakeV2Subsystem.setArmLiftState(armLiftState);
//            intakeV2Subsystem.setClawRotState(clawRotState);
//            intakeV2Subsystem.setClawState(clawState);
//
////            if(raise_enabled) {
////                intakeV2Subsystem.raiseServoL.setPosition(raiseServoPos);
////                intakeV2Subsystem.raiseServoR.setPosition(1-raiseServoPos+raise_off);
////            } else {
////                intakeV2Subsystem.raiseServoL.setPwmDisable();
////                intakeV2Subsystem.raiseServoR.setPwmDisable();
////            }
////
////            if(arm_rot_enabled) {
////                intakeV2Subsystem.arm_rot.setPosition(armRotPos);
////            } else {
////                intakeV2Subsystem.arm_rot.setPwmDisable();
////            }
////
////            if(arm_lift_enabled) {
////                intakeV2Subsystem.arm_lift.setPosition(armLiftPos);
////            } else {
////                intakeV2Subsystem.arm_lift.setPwmDisable();
////            }
////
////            if(claw_rot_enabled) {
////                intakeV2Subsystem.claw_rot.setPosition(clawRotPos);
////            } else {
////                intakeV2Subsystem.claw_rot.setPwmDisable();
////            }
//        }
//    }
}
