package org.firstinspires.ftc.teamcode.IntoTheDeepRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.CouplersSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.drive.DriveConstants;

public class IntoTheDeepRobot extends RobotEx {
    protected RobotMap robotMap;
    //----------------------------------- Initialize Subsystems ----------------------------------//
    protected ClawSubsystem clawSubsystem;
    protected ArmSubsystem armSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected ElevatorSubsystem elevatorSubsystem;
    protected CouplersSubsystem couplersSubsystem;

    public IntoTheDeepRobot(RobotMap robotMap, DriveConstants RobotConstants,
                            OpModeType opModeType, Alliance alliance, boolean init_camera,
                            Pose2d startingPose) {
        super(robotMap, RobotConstants, opModeType, alliance, init_camera, startingPose);
        this.robotMap = robotMap;
        this.initMechanismsTeleOp();
    }

    @Override
    public void initMechanismsTeleOp() {
        clawSubsystem = new ClawSubsystem(this.robotMap);
        armSubsystem = new ArmSubsystem(this.robotMap);
//        intakeSubsystem = new IntakeSubsystem(this.robotMap);
        elevatorSubsystem = new ElevatorSubsystem(this.robotMap, () -> toolOp.getRightY());
//        couplersSubsystem = new CouplersSubsystem(this.robotMap);

        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                new InstantCommand(clawSubsystem::grab),
                new InstantCommand(clawSubsystem::release),
                () -> clawSubsystem.getState() == ClawSubsystem.ClawState.OPEN
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ConditionalCommand(
                new InstantCommand(clawSubsystem::goFlipped),
                new InstantCommand(clawSubsystem::goNormal),
                () -> clawSubsystem.getRotState() == ClawSubsystem.ClawRotState.NORMAL
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(armSubsystem::wrist_goIntake),
                        new WaitCommand(130),
                        new InstantCommand(armSubsystem::arm_goIntake)
                )
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(armSubsystem::wrist_goPark),
                        new WaitCommand(130),
                        new InstantCommand(armSubsystem::arm_goPark)
                )
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(armSubsystem::wrist_goHigh),
                        new WaitCommand(130),
                        new InstantCommand(armSubsystem::arm_goHigh)
                )
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(armSubsystem::wrist_goPerp),
                        new WaitCommand(130),
                        new InstantCommand(armSubsystem::arm_goPerp)
                )
        );

        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(armSubsystem::wrist_goIntakeBAbove),
                        new WaitCommand(130),
                        new InstantCommand(armSubsystem::arm_goIntakeBAbove)
                )
        );

        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(armSubsystem::wrist_goIntakeB),
                        new WaitCommand(130),
                        new InstantCommand(armSubsystem::arm_goIntakeB)
                )
        );
    }
}
