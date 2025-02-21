package org.firstinspires.ftc.teamcode.IntoTheDeepRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.CouplersSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.DistanceSensorsSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.HangingSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.ForwardControllerSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.HeadingControllerSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.StrafeControllerSubsystem;
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
    protected HangingSubsystem hangingSubsystem;
    protected CouplersSubsystem couplersSubsystem;
    protected DistanceSensorsSubsystem distanceSensorsSubsystem;

    // ---------------------------------- Initialize Controllers -------------------------------- //
//    protected ForwardControllerSubsystem forwardController;
    protected StrafeControllerSubsystem strafeControllerSubsystem;
    protected HeadingControllerSubsystem gyroFollow;

    public IntoTheDeepRobot(RobotMap robotMap, DriveConstants RobotConstants,
                            OpModeType opModeType, Alliance alliance, boolean init_camera,
                            Pose2d startingPose) {
        super(robotMap, RobotConstants, opModeType, alliance, init_camera, startingPose);
        this.robotMap = robotMap;
        this.initMechanismsTeleOp();
    }

    public SequentialCommandGroup intake_sample() {
        return new SequentialCommandGroup(
                // Go to Park State: Elevator, Arm, Wrist, Claw
                new InstantCommand(
                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                ),
                new ConditionalCommand(
                        new InstantCommand(() -> extendoSubsystem.setTargetPosition(200), extendoSubsystem),
                        new InstantCommand(),
                        () -> extendoSubsystem.getExtension() < 200
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> armSubsystem.setWristState(
                                        ArmSubsystem.WristState.INTAKE
                                )),
                                new InstantCommand(clawSubsystem::goNormal, clawSubsystem),
                                new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                                new WaitCommand(120),
                                new InstantCommand(() -> armSubsystem.setArmState(
                                        ArmSubsystem.ArmState.INTAKE
                                ))
                        ),
                        // Intake Procedure
                        new IntakeCommand(
                                intakeSubsystem,
                                (this.getAlliance() == Alliance.RED ?
                                        IntakeCommand.COLOR.RED_YELLOW : IntakeCommand.COLOR.BLUE_YELLOW)
                        )
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> extendoSubsystem.blockManual(true)),
                                new InstantCommand(
                                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                                ),
                                new InstantCommand(() -> extendoSubsystem.setTargetPosition(90), extendoSubsystem),
                                // Push Sample (Align to Parrot)
                                new InstantCommand(intakeSubsystem::run),
                                new WaitCommand(120),
                                new InstantCommand(intakeSubsystem::stop),
                                new InstantCommand(intakeSubsystem::raise, intakeSubsystem),
                                new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
                                new WaitUntilCommand(() -> elevatorSubsystem.atTarget()),
                                new InstantCommand(clawSubsystem::grab),
                                new WaitCommand(60),
                                // Disengage Sample from the Intake/Parrot
                                new InstantCommand(
                                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.PARK2)
                                ),
                                new InstantCommand(() -> extendoSubsystem.blockManual(false))
                        ),
                        discard_sample(),
                        () -> intakeSubsystem.check_color(this.getAlliance())
                )
        );
    }

    public SequentialCommandGroup discard_sample() {
        return new SequentialCommandGroup(
                new InstantCommand(intakeSubsystem::raise),
                new InstantCommand(intakeSubsystem::reverse),
                new WaitCommand(1000),
                new InstantCommand(intakeSubsystem::stop),
                new InstantCommand(intakeSubsystem::lower)
        );
    }

    @Override
    public void initMechanismsTeleOp() {
        clawSubsystem = new ClawSubsystem(this.robotMap);
        armSubsystem = new ArmSubsystem(this.robotMap);
        intakeSubsystem = new IntakeSubsystem(this.robotMap);
        elevatorSubsystem = new ElevatorSubsystem(
                this.robotMap,
                () -> -toolOp.getRightY(),
                robotMap.getRearLeftMotor(),
                telemetry,
                true
        );
        extendoSubsystem = new ExtendoSubsystem(
                this.robotMap,
                () -> toolOp.getLeftY(),
                telemetry,
                true
        );
        hangingSubsystem = new HangingSubsystem(this.robotMap);
        couplersSubsystem = new CouplersSubsystem(this.robotMap);
        distanceSensorsSubsystem = new DistanceSensorsSubsystem(this.robotMap, telemetry);

//        forwardController = new ForwardControllerSubsystem(
//                () -> distanceSensorsSubsystem.getDistances()[0],
//                dashboard.getTelemetry()
//        );

        strafeControllerSubsystem = new StrafeControllerSubsystem(
                () -> distanceSensorsSubsystem.getDistances()[2],
                dashboard.getTelemetry()
        );

        gyroFollow = new HeadingControllerSubsystem(
                this::getContinuousHeading,
                () -> 0,
                dashboard.getTelemetry()
        );

        //------------------------------------ Manual Actions ---------------------------------- //
//         Claw Grab/Release(Just Open) Toggle
        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ConditionalCommand(
                new InstantCommand(clawSubsystem::grab),
                new InstantCommand(clawSubsystem::justOpen),
                () -> clawSubsystem.getState() != ClawSubsystem.ClawState.CLOSED
        ));

        // Claw Rot Normal/Flipped Toggle
        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> elevatorSubsystem.setLevel(
                        ElevatorSubsystem.Level.PARK2)
                ),
                new WaitCommand(300),
                new InstantCommand(() -> armSubsystem.setWristState(
                        ArmSubsystem.WristState.HUMAN_PLAYER
                )),
                new WaitCommand(100),
                new InstantCommand(() -> armSubsystem.setArmState(
                        ArmSubsystem.ArmState.HUMAN_PLAYER
                )),
                new InstantCommand(() -> elevatorSubsystem.setLevel(
                        ElevatorSubsystem.Level.PARK
                )),
                new WaitCommand(1000),
                new InstantCommand(clawSubsystem::release),
                new WaitCommand(200),
                new InstantCommand(() -> elevatorSubsystem.setLevel(
                        ElevatorSubsystem.Level.PARK)
                ),
                new WaitCommand(250),
                new InstantCommand(clawSubsystem::goNormal, clawSubsystem),
                new InstantCommand(() -> armSubsystem.setWristState(
                        ArmSubsystem.WristState.SPECIMENT_INTAKE
                )),
                new WaitCommand(120),
                new InstantCommand(() -> armSubsystem.setArmState(
                        ArmSubsystem.ArmState.SPECIMENT_INTAKE
                )),
                new WaitCommand(200),
                new InstantCommand(() -> elevatorSubsystem.setLevel(
                        ElevatorSubsystem.Level.INTAKE
                ))
        ));

        // Intake Raise/Lower Toggle
        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                new InstantCommand(intakeSubsystem::raise),
                new InstantCommand(intakeSubsystem::lower),
                () -> intakeSubsystem.getRaiseState() == IntakeSubsystem.RaiseState.LOWERED
        ));

        // Coupler Engage/Disengage Toggle
        new Trigger(
                () -> toolOp.getGamepadButton(GamepadKeys.Button.BACK).get() &&
                        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get())
                .whenActive(new ConditionalCommand(
                        new InstantCommand(couplersSubsystem::disengage),
                        new InstantCommand(couplersSubsystem::engage),
                        () -> couplersSubsystem.getState() == CouplersSubsystem.CouplerState.ENGAGED
                ));

        toolOp.getGamepadButton(GamepadKeys.Button.START).whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> this.drive_setEnabled(false)),
                new InstantCommand(hangingSubsystem::ascend
        ))
        ).whenReleased(
            new SequentialCommandGroup(
                new InstantCommand(() -> this.drive_setEnabled(true)),
                new InstantCommand(hangingSubsystem::stop)
            )
        );

        // ------------------------------------ Automatations ----------------------------------- //
        //// Intake Specimen Automation
        toolOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> elevatorSubsystem.setLevel(
                                        ElevatorSubsystem.Level.PARK)
                                ),
                                new WaitCommand(250),
                                new InstantCommand(clawSubsystem::goFlipped, clawSubsystem),
                                new InstantCommand(() -> armSubsystem.setWristState(
                                        ArmSubsystem.WristState.SPECIMENT_INTAKE
                                )),
                                new WaitCommand(120),
                                new InstantCommand(() -> armSubsystem.setArmState(
                                        ArmSubsystem.ArmState.SPECIMENT_INTAKE
                                )),
                                new WaitCommand(200),
                                new InstantCommand(() -> elevatorSubsystem.setLevel(
                                        ElevatorSubsystem.Level.INTAKE
                                )),
                                new InstantCommand(clawSubsystem::release)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(clawSubsystem::grab),
                                new WaitCommand(200),
                                new InstantCommand(() -> elevatorSubsystem.setLevel(
                                        ElevatorSubsystem.Level.SPECIMEN_DISLOCATE
                                ))
                        ),
                        () -> armSubsystem.getArmState() != ArmSubsystem.ArmState.SPECIMENT_INTAKE
                )
        );
        //// Intake Sample Automation
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).toggleWhenPressed(intake_sample());

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(discard_sample());

        //// Elevator Height Automations ++ Arm
        // Low Basket
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(
                            () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.LOW_BASKET)
                    ),
                    new WaitCommand(200),
                    new InstantCommand(() -> armSubsystem.setWristState(
                            ArmSubsystem.WristState.BASKET_OUTTAKE
                    )),
                    new WaitCommand(120),
                    new InstantCommand(() -> armSubsystem.setArmState(
                            ArmSubsystem.ArmState.BASKET_OUTTAKE
                    ))
                )
        );

        // High Basket
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(
                            () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.HIGH_BASKET)
                    ),
                    new WaitCommand(200),
                    new InstantCommand(() -> armSubsystem.setWristState(
                            ArmSubsystem.WristState.BASKET_OUTTAKE
                    )),
                    new WaitCommand(120),
                    new InstantCommand(() -> armSubsystem.setArmState(
                            ArmSubsystem.ArmState.BASKET_OUTTAKE
                    ))
                )
        );

        // Low Chamber
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(
                            () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.LOW_CHAMBER)
                    ),
                    new WaitCommand(50),
                    new InstantCommand(() -> armSubsystem.setWristState(
                            ArmSubsystem.WristState.SPECIMENT_OUTTAKE_LOW
                    )),
                    new WaitCommand(120),
                    new InstantCommand(() -> armSubsystem.setArmState(
                            ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_LOW
                    ))
                )
        );

        // High Chamber
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(clawSubsystem::goNormal),
                        new InstantCommand(() -> elevatorSubsystem.setLevel(
                                ElevatorSubsystem.Level.HIGH_CHAMBER
                        )),
                        new WaitCommand(150),
                        new InstantCommand(() -> armSubsystem.setWristState(
                                ArmSubsystem.WristState.SPECIMEN_OUTTAKE
                        )),
                        new WaitCommand(100),
                        new InstantCommand(() -> armSubsystem.setArmState(
                                ArmSubsystem.ArmState.SPECIMEN_OUTTAKE
                        ))
                )
        );

        //// Basket Outtake Automation
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.4)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(clawSubsystem::release),
                        new WaitCommand(1200),
                        new InstantCommand(() -> armSubsystem.setWristState(
                                ArmSubsystem.WristState.PARK
                        )),
                        new WaitCommand(120),
                        new InstantCommand(
                                () -> armSubsystem.setArmState(ArmSubsystem.ArmState.PARK)
                        ),
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.PARK)
                        )
                ));

        //// Chamber Outtake Automation
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.4)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> elevatorSubsystem.setLevel(
                                ElevatorSubsystem.Level.HIGH_CHAMBER_RELEASE
                        )),
                        new WaitUntilCommand(() -> elevatorSubsystem.atTarget()),
                        new InstantCommand(clawSubsystem::release)
                ));

        // Hanging Automation
        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.HANGING_AIM)
                        ),
                        new InstantCommand(intakeSubsystem::raise, intakeSubsystem),
                        new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
                        new InstantCommand(clawSubsystem::grab, clawSubsystem),
                        new InstantCommand(extendoSubsystem::returnToZero, extendoSubsystem),
                        new InstantCommand(() -> armSubsystem.setWristState(
                                ArmSubsystem.WristState.INTAKE
                        )),
                        new WaitCommand(120),
                        new InstantCommand(() -> armSubsystem.setArmState(
                                ArmSubsystem.ArmState.INTAKE
                        ))
                ),
                new SequentialCommandGroup( // Ascending Cmd
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.HANGING)
                        ),
                        new InstantCommand(extendoSubsystem::returnToZero, extendoSubsystem),
                        new InstantCommand(intakeSubsystem::hang, intakeSubsystem),
                        new InstantCommand(() -> this.drive_setEnabled(false)),
                        new WaitUntilCommand(
                                () -> elevatorSubsystem.getHeight() < 350
                        ),
                        new InstantCommand(couplersSubsystem::engage, couplersSubsystem),
                        new InstantCommand(()->elevatorSubsystem.setCoupled(true)),
                        new WaitUntilCommand(
                                () -> elevatorSubsystem.getHeight() < 70
                        ),
                        new InstantCommand(hangingSubsystem::release, hangingSubsystem),
                        new WaitCommand(1500),
                        new InstantCommand(hangingSubsystem::ascend, hangingSubsystem),
                        new WaitCommand(1200),
                        new InstantCommand(elevatorSubsystem::disable, elevatorSubsystem),
                        new WaitUntilCommand(() -> hangingSubsystem.reached_hanging_pos()),
                        new InstantCommand(hangingSubsystem::descend, hangingSubsystem),
                        new WaitCommand(300),
                        new InstantCommand(hangingSubsystem::stop, hangingSubsystem)
                ),
                () -> elevatorSubsystem.getLevel() != ElevatorSubsystem.Level.HANGING_AIM
        ));

        // Specimen Outtake Automation with Distance Sensor (6.3, 13)
        new Trigger(
                () -> distanceSensorsSubsystem.getDistances()[0] <=
                        (armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_LOW ? 6.7 : 13) &&
                        (armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_LOW ||
                                armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_HIGH)
        ).whenActive(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                        new WaitCommand(150),
                        new InstantCommand(
                                () -> armSubsystem.setArmState(ArmSubsystem.ArmState.PERP)
                        )
                ),
                new SequentialCommandGroup(
                        new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                        new WaitCommand(150),
                        new InstantCommand(
                                () -> armSubsystem.setArmState(ArmSubsystem.ArmState.INTAKE_B)
                        )
                ),
                () -> armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_HIGH
        ));

        // ------------------------------------ Drive Commands ---------------------------------- //
        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(
                        new SequentialCommandGroup(
                            new InstantCommand(gyroFollow::enable),
                            new InstantCommand(() -> gyroFollow.setGyroTarget(180))
                        )
                ).whenReleased(
                        new InstantCommand(gyroFollow::disable)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(this::setRobotCentric),
                                new InstantCommand(gyroFollow::enable),
                                new InstantCommand(strafeControllerSubsystem::enable),
                                new InstantCommand(() -> gyroFollow.setGyroTarget(0)),
                                new InstantCommand(() -> strafeControllerSubsystem.setDistTarget(39.7))
                        )
                ).whenReleased(new SequentialCommandGroup(
                        new InstantCommand(this::setFieldCentric),
                        new InstantCommand(gyroFollow::disable),
                        new InstantCommand(strafeControllerSubsystem::disable)
                ));
    }

//    @Override
//    public double drivetrainForward() {
//        return strafeControllerSubsystem.calculatePower();
//    }

    @Override
    public double drivetrainStrafe() {
//        if (strafeControllerSubsystem.isEnabled()) return strafeControllerSubsystem.calculatePower();

        return super.drivetrainStrafe();
    }

    @Override
    public double drivetrainTurn() {
//        if (gyroFollow.isEnabled()) return -gyroFollow.calculateTurn();

        return super.drivetrainTurn();
    }
}
