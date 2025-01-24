package org.firstinspires.ftc.teamcode.IntoTheDeepRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.CouplersSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ExtendoSubsystem;
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
    protected ExtendoSubsystem extendoSubsystem;
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
        intakeSubsystem = new IntakeSubsystem(this.robotMap);
        elevatorSubsystem = new ElevatorSubsystem(
                this.robotMap,
                () -> -toolOp.getRightY(),
                telemetry
        );
        extendoSubsystem = new ExtendoSubsystem(this.robotMap, () -> toolOp.getLeftY(), telemetry);
        couplersSubsystem = new CouplersSubsystem(this.robotMap);

        // ------------------------------------ Manual Actions ---------------------------------- //
        // Claw Grab/Release(Just Open) Toggle
        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ConditionalCommand(
                new InstantCommand(clawSubsystem::grab),
                new InstantCommand(clawSubsystem::justOpen),
                () -> clawSubsystem.getState() != ClawSubsystem.ClawState.CLOSED
        ));

        // Claw Rot Normal/Flipped Toggle
        toolOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ConditionalCommand(
                new InstantCommand(clawSubsystem::goFlipped),
                new InstantCommand(clawSubsystem::goNormal),
                () -> clawSubsystem.getRotState() == ClawSubsystem.ClawRotState.NORMAL
        ));

        // Intake Raise/Lower Toggle
        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                new InstantCommand(intakeSubsystem::raise),
                new InstantCommand(intakeSubsystem::lower),
                () -> intakeSubsystem.getRaiseState() == IntakeSubsystem.RaiseState.LOWERED
        ));

        // Elevator Intake/Park Toggle
        toolOp.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(
                new InstantCommand(
                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                ),
                new InstantCommand(
                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.PARK)
                )
        );

        // Coupler Engage/Disengage Toggle
        new Trigger(
                () -> toolOp.getGamepadButton(GamepadKeys.Button.START).get() &&
                        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get())
                .whenActive(new ConditionalCommand(
                        new InstantCommand(couplersSubsystem::disengage),
                        new InstantCommand(couplersSubsystem::engage),
                        () -> couplersSubsystem.getState() == CouplersSubsystem.CouplerState.ENGAGED
                ));

        // ------------------------------------ Automatations ----------------------------------- //
        //// Intake Automation
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(
                new SequentialCommandGroup(
//                        // Lower Intake Arm System
//                        new InstantCommand(intakeSubsystem::lower),
//                        new WaitCommand(80),
                        // Go to Park State: Elevator, Arm, Wrist, Claw
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.PARK)
                        ),

                        new InstantCommand(armSubsystem::wrist_goIntake),
                        new InstantCommand(clawSubsystem::goNormal, clawSubsystem),
                        new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                        new WaitCommand(120),
                        new InstantCommand(armSubsystem::arm_goIntake),
                        // Intake Procedure
                        new IntakeCommand(
                                intakeSubsystem,
                                IntakeCommand.COLOR.RED_YELLOW
                        ),
                        new InstantCommand(intakeSubsystem::brake_reverse),
                        new WaitCommand(120),
                        new InstantCommand(intakeSubsystem::stop),
                        new WaitCommand(50),
                        // Raise Intake and Return Extendo
                        new InstantCommand(intakeSubsystem::raise, intakeSubsystem),
                        new InstantCommand(extendoSubsystem::returnToZero, extendoSubsystem),
                        new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
                        // Push Sample (Align to Parrot)
                        new InstantCommand(intakeSubsystem::run),
                        new WaitCommand(260),
                        new InstantCommand(intakeSubsystem::stop),
                        new WaitCommand(50),
                        // Lower Slider w/ Claw and grab Sample
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                        ),
                        new WaitCommand(200), // TODO Check if this is necessary
                        new InstantCommand(clawSubsystem::grab),
                        new WaitCommand(400),
                        // Disengage Sample from the Intake/Parrot
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.PARK2)
                        )
                )
        );

        //// Elevator Height Automations ++ Arm
        // Low Basket
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(
                            () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.LOW_BASKET)
                    ),
                    new WaitCommand(200),
                    new InstantCommand(armSubsystem::wrist_goBasketOuttake),
                    new WaitCommand(120),
                    new InstantCommand(armSubsystem::arm_goBasketOuttake)
                )
        );

        // High Basket
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(
                            () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.HIGH_BASKET)
                    ),
                    new WaitCommand(200),
                    new InstantCommand(armSubsystem::wrist_goBasketOuttake),
                    new WaitCommand(120),
                    new InstantCommand(armSubsystem::arm_goBasketOuttake)
                )
        );

        // Low Chamber
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(
                            () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.LOW_CHAMBER)
                    ),
                    new WaitCommand(200),
                    new InstantCommand(armSubsystem::wrist_goSpecimentOuttakeLow),
                    new WaitCommand(120),
                    new InstantCommand(armSubsystem::arm_goSpecimentOuttakeLow)
                )
        );

        // High Chamber
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(
                            () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.HIGH_CHAMBER)
                    ),
                    new WaitCommand(200),
                    new InstantCommand(armSubsystem::wrist_goSpecimentOuttake),
                    new WaitCommand(120),
                    new InstantCommand(armSubsystem::arm_goSpecimentOuttake)
                )
        );

        //// Basket Outtake Automation
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.4)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(clawSubsystem::release),
                        new WaitCommand(1200),
                        new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                        new InstantCommand(armSubsystem::wrist_goPark),
                        new WaitCommand(120),
                        new InstantCommand(armSubsystem::arm_goPark),
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.PARK)
                        )
                ));

        //// Chamber Outtake Automation
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.4)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(armSubsystem::arm_goPerp),
                        new WaitCommand(200),
                        new InstantCommand(clawSubsystem::justOpen, clawSubsystem)
                ));

        // Hanging Automation
        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ConditionalCommand(
                new InstantCommand(
                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.HANGING_AIM)
                ),
                new SequentialCommandGroup( // Ascending Cmd
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.HANGING)
                        )
                ),
                () -> elevatorSubsystem.getLevel() != ElevatorSubsystem.Level.HANGING_AIM
        ));
    }
}
