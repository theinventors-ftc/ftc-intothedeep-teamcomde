package org.firstinspires.ftc.teamcode.Tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;

@TeleOp(name = "Wiper Test", group = "Tests")
public class WiperTest extends CommandOpMode {

    private RobotMap robotMap;
    private IntakeSubsystem intakeSubsystem;

    private SequentialCommandGroup temp;

    private RobotEx.Alliance alliance = RobotEx.Alliance.RED;

    public void initialize() {
        CommandScheduler.getInstance().reset();
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2, RobotMap.OpMode.TELEOP);
        intakeSubsystem = new IntakeSubsystem(robotMap);
    }

    @Override
    public void run() {

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.b) {
                new InstantCommand(intakeSubsystem::contract);
            }

            if (gamepad1.a) {
                new InstantCommand(intakeSubsystem::full_open);
            }

            if (gamepad1.x) {
                new InstantCommand(intakeSubsystem::semi_open);
            }
        }
    }
}
