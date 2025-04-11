package org.firstinspires.ftc.teamcode.IntoTheDeepRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.PedroSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.HeadingControllerSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.CouplersSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.DistanceSensorsSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.HangingSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;

import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.drive.DriveConstants;

public class IntoTheDeepRobot extends RobotEx {
    protected RobotMap robotMap;
    protected Follower follower;
    //----------------------------------- Initialize Subsystems ----------------------------------//
    protected ClawSubsystem clawSubsystem;
    protected ArmSubsystem armSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected ElevatorSubsystem elevatorSubsystem;
    protected ExtendoSubsystem extendoSubsystem;
    protected HangingSubsystem hangingSubsystem;
    protected CouplersSubsystem couplersSubsystem;
    protected DistanceSensorsSubsystem distanceSensorsSubsystem;
    protected PedroSubsystem pedroSubsystem;

    // ---------------------------------- Initialize Controllers -------------------------------- //
//    protected ForwardControllerSubsystem forwardController;
//    protected StrafeControllerSubsystem strafeControllerSubsystem;
    protected HeadingControllerSubsystem gyroFollow;

    private boolean hasInit = false;

    public IntoTheDeepRobot(RobotMap robotMap, DriveConstants RobotConstants,
                            OpModeType opModeType, Alliance alliance, boolean init_camera,
                            Pose2d startingPose) {
        super(robotMap, RobotConstants, opModeType, alliance, init_camera, startingPose);
        this.robotMap = robotMap;

        new Trigger(() -> (Math.abs(drivetrainForward()) > 0.1 ||
                Math.abs(drivetrainStrafe()) > 0.1 ||
                Math.abs(drivetrainTurn()) > 0.1) && !hasInit)
                .whenActive(new InstantCommand(this::initMechanismsTeleOp));
    }

    public IntoTheDeepRobot(Follower follower, RobotMap robotMap,
                            DriveConstants RobotConstants,
                            OpModeType opModeType, Alliance alliance, boolean init_camera,
                            Pose2d startingPose) {
        super(robotMap, RobotConstants, opModeType, alliance, init_camera, startingPose);
        this.robotMap = robotMap;
        this.follower = follower;

        new Trigger(() -> (Math.abs(drivetrainForward()) > 0.1 ||
            Math.abs(drivetrainStrafe()) > 0.1 ||
            Math.abs(drivetrainTurn()) > 0.1) && !hasInit)
            .whenActive(new InstantCommand(this::initMechanismsTeleOp));
    }

    public SequentialCommandGroup specimenAim() {
        return new SequentialCommandGroup( // PEOS INTAKE AIM
            new InstantCommand(() -> elevatorSubsystem.setLevel(
               ElevatorSubsystem.Level.SPEC_INTAKE)
            ),
            new InstantCommand(() -> armSubsystem.setArmState(
               ArmSubsystem.ArmState.PARK
            )),
            new InstantCommand(() -> armSubsystem.setWristState(
               ArmSubsystem.WristState.PARK
            )),
            new WaitCommand(200),
            new InstantCommand(() -> armSubsystem.setWristState(
               ArmSubsystem.WristState.SPEC_INTAKE_NEW
            )),
            new WaitCommand(100),
            new InstantCommand(() -> armSubsystem.setArmState(
               ArmSubsystem.ArmState.SPEC_INTAKE_NEW
            )),
            new InstantCommand(clawSubsystem::release),
            new InstantCommand(extendoSubsystem::returnToZero)
        );
    }

    public SequentialCommandGroup scoreSpeciment() {
        return new SequentialCommandGroup(
            new InstantCommand(clawSubsystem::firmlyGripped),
            new WaitCommand(200),
            new InstantCommand(() -> elevatorSubsystem.setLevel(
                ElevatorSubsystem.Level.SPEC_INTAKE
            )),
            new InstantCommand(() -> armSubsystem.setWristState(
                ArmSubsystem.WristState.SPEC_OUTTAKE_AIM_NEW
            )),
            new InstantCommand(() -> armSubsystem.setArmState(
                ArmSubsystem.ArmState.SPEC_OUTTAKE_AIM_NEW
            ))
        );
    }

    public SequentialCommandGroup releaseSpecimen() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> armSubsystem.setArmState(
                ArmSubsystem.ArmState.SPEC_OUTTAKE_NEW
            )),
            new InstantCommand(() -> armSubsystem.setWristState(
                ArmSubsystem.WristState.SPEC_OUTTAKE_NEW
            )),
            new WaitCommand(300),
            new InstantCommand(clawSubsystem::release)
        );
    }

    public SequentialCommandGroup intake_sample_for_specimen() {
        return new SequentialCommandGroup(
                // Go to Park State: Elevator, Arm, Wrist, Claw
                new InstantCommand(
                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                ),
                new ConditionalCommand(
                        new InstantCommand(() -> extendoSubsystem.setTargetPosition(400), extendoSubsystem),
                        new InstantCommand(),
                        () -> extendoSubsystem.getExtension() < 400
                ),
//                new InstantCommand(intakeSubsystem::wiper_semi_open),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> armSubsystem.setWristState(
                                        ArmSubsystem.WristState.INTAKE
                                )),
                                new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                                new WaitCommand(70),
                                new InstantCommand(() -> armSubsystem.setArmState(
                                        ArmSubsystem.ArmState.INTAKE
                                ))
                        ),
                        // Intake Procedure
                        new IntakeCommand(
                                intakeSubsystem,
                                (this.getAlliance() == Alliance.RED ?
                                        IntakeCommand.COLOR.RED_YELLOW : IntakeCommand.COLOR.BLUE_YELLOW),
                                extendoSubsystem
                        )
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(intakeSubsystem::stop),
//                                new InstantCommand(intakeSubsystem::wiper_contract),
                                new InstantCommand(() -> extendoSubsystem.blockManual(true)),
                                new InstantCommand(
                                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                                ),
                                new InstantCommand(() -> extendoSubsystem.setTargetPosition(140), extendoSubsystem),
                                new InstantCommand(intakeSubsystem::raise, intakeSubsystem),
                                new InstantCommand(() -> extendoSubsystem.blockManual(false))
                        ),
                        discard_sample(),
                        () -> intakeSubsystem.check_color(this.getAlliance(), false)
                )
        );
    }

    public SequentialCommandGroup intake_sample() {
        return new SequentialCommandGroup(
                // Go to Park State: Elevator, Arm, Wrist, Claw
                new InstantCommand(
                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                ),
                new ConditionalCommand(
                        new InstantCommand(() -> extendoSubsystem.setTargetPosition(250), extendoSubsystem),
                        new InstantCommand(),
                        () -> extendoSubsystem.getExtension() < 250
                ),
//                new InstantCommand(intakeSubsystem::wiper_semi_open),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> armSubsystem.setWristState(
                                        ArmSubsystem.WristState.INTAKE
                                )),
                                new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                                new WaitCommand(70),
                                new InstantCommand(() -> armSubsystem.setArmState(
                                        ArmSubsystem.ArmState.INTAKE
                                ))
                        ),
                        // Intake Procedure
                        new IntakeCommand(
                                intakeSubsystem,
                                (this.getAlliance() == Alliance.RED ?
                                        IntakeCommand.COLOR.RED_YELLOW : IntakeCommand.COLOR.BLUE_YELLOW),
                                extendoSubsystem
                        )
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> extendoSubsystem.blockManual(true)),
                                new InstantCommand(
                                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                                ),
                                new InstantCommand(extendoSubsystem::returnToZero, extendoSubsystem),
                                new InstantCommand(intakeSubsystem::raise, intakeSubsystem),
                                // Push Sample (Align to Parrot)
                                new InstantCommand(intakeSubsystem::run),
                                new WaitCommand(120),
                                new InstantCommand(intakeSubsystem::stop),
//                                new InstantCommand(intakeSubsystem::wiper_contract),
                                new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
                                new WaitUntilCommand(() -> elevatorSubsystem.atTarget()),
                                new WaitCommand(30),
                                new InstantCommand(clawSubsystem::looslyGripped),
                                new WaitCommand(60),
                                new InstantCommand(intakeSubsystem::reverse),
                                // Disengage Sample from the Intake/Parrot
                                new InstantCommand(
                                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.PARK2)
                                ),
                                new InstantCommand(() -> extendoSubsystem.blockManual(false)),
                                new WaitCommand(350),
                                new InstantCommand(intakeSubsystem::stop),
                                new InstantCommand(clawSubsystem::firmlyGripped)
                        ),
                        discard_sample(),
                        () -> intakeSubsystem.check_color(this.getAlliance(), true)
                )
        );
    }

    public SequentialCommandGroup discard_sample() {
        return new SequentialCommandGroup(
                new InstantCommand(intakeSubsystem::raise),
                new InstantCommand(intakeSubsystem::reverse),
                new WaitCommand(800),
                new InstantCommand(intakeSubsystem::stop)
        );
    }

    @Override
    public void initMechanismsTeleOp() {
        hasInit = true;
        drive_isInAuto(false);

        clawSubsystem = new ClawSubsystem(this.robotMap);
        armSubsystem = new ArmSubsystem(this.robotMap);
        intakeSubsystem = new IntakeSubsystem(this.robotMap);
        elevatorSubsystem = new ElevatorSubsystem(
                this.robotMap,
                () -> -toolOp.getRightY(),
                robotMap.getRearLeftMotor(),
                telemetry,
                false // TODO: These are wrong
        );
        extendoSubsystem = new ExtendoSubsystem(
                this.robotMap,
                () -> toolOp.getLeftY(),
                telemetry,
                false // TODO: These are wrong
        );
        hangingSubsystem = new HangingSubsystem(this.robotMap);
        couplersSubsystem = new CouplersSubsystem(this.robotMap);
        distanceSensorsSubsystem = new DistanceSensorsSubsystem(this.robotMap, telemetry);

//        forwardController = new ForwardControllerSubsystem(
//                () -> distanceSensorsSubsystem.getDistances()[0],
//                dashboard.getTelemetry()
//        );

//        strafeControllerSubsystem = new StrafeControllerSubsystem(
//                () -> distanceSensorsSubsystem.getDistances()[2],
//                dashboard.getTelemetry()
//        );

        gyroFollow = new HeadingControllerSubsystem(
                this::getContinuousHeading,
                () -> 0,
                dashboard.getTelemetry()
        );

        pedroSubsystem = new PedroSubsystem(follower,
                                            scoreSpeciment(),
                                            releaseSpecimen(),
                                            specimenAim());

        //------------------------------------ Manual Actions ---------------------------------- //
//         Claw Grab/Release(Just Open) Toggle
        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ConditionalCommand(
                new InstantCommand(clawSubsystem::firmlyGripped),
                new InstantCommand(clawSubsystem::justOpen),
                () -> clawSubsystem.getState() != ClawSubsystem.ClawState.FIRMLY_GRIPPED
        ));
//        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ConditionalCommand(
//                new InstantCommand(couplersSubsystem::engage),
//                new InstantCommand(couplersSubsystem::disengage),
//                () -> couplersSubsystem.getState() != CouplersSubsystem.CouplerState.ENGAGED
//        ));

        toolOp.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(intake_sample_for_specimen());

        // Intake Raise/Lower Toggle
        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                new InstantCommand(intakeSubsystem::raise),
                new InstantCommand(intakeSubsystem::lower),
                () -> intakeSubsystem.getRaiseState() == IntakeSubsystem.RaiseState.LOWERED
        ));

        // Coupler Engage/Disengage Toggle
//        new Trigger(
//                () -> toolOp.getGamepadButton(GamepadKeys.Button.START).get() &&
//                        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).get())
//                .whenActive(new ConditionalCommand(
//                        new InstantCommand(couplersSubsystem::disengage),
//                        new InstantCommand(couplersSubsystem::engage),
//                        () -> couplersSubsystem.getState() == CouplersSubsystem.CouplerState.ENGAGED
//                ));

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
                        new SequentialCommandGroup( // PEOS INTAKE AIM
                                new InstantCommand(() -> elevatorSubsystem.setLevel(
                                        ElevatorSubsystem.Level.SPEC_INTAKE)
                                ),
                                new InstantCommand(() -> armSubsystem.setArmState(
                                        ArmSubsystem.ArmState.PARK
                                )),
                                new InstantCommand(() -> armSubsystem.setWristState(
                                        ArmSubsystem.WristState.PARK
                                )),
                                new WaitCommand(200),
                                new InstantCommand(() -> armSubsystem.setWristState(
                                        ArmSubsystem.WristState.SPEC_INTAKE_NEW
                                )),
                                new WaitCommand(100),
                                new InstantCommand(() -> armSubsystem.setArmState(
                                        ArmSubsystem.ArmState.SPEC_INTAKE_NEW
                                )),
                                new InstantCommand(clawSubsystem::release),
                                new InstantCommand(extendoSubsystem::returnToZero)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(clawSubsystem::firmlyGripped),
                                new WaitCommand(200),
                                new InstantCommand(() -> elevatorSubsystem.setLevel(
                                        ElevatorSubsystem.Level.SPEC_INTAKE
                                )),
                                new InstantCommand(() -> armSubsystem.setWristState(
                                        ArmSubsystem.WristState.SPEC_OUTTAKE_AIM_NEW
                                )),
                                new InstantCommand(() -> armSubsystem.setArmState(
                                        ArmSubsystem.ArmState.SPEC_OUTTAKE_AIM_NEW
                                ))
//                                new WaitCommand(400),
//                                new InstantCommand(() -> elevatorSubsystem.setLevel(
//                                        ElevatorSubsystem.Level.SPEC_MIDPOINT
//                                ))
                        ),
                        () -> armSubsystem.getArmState() != ArmSubsystem.ArmState.SPEC_INTAKE_NEW
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
                        new InstantCommand(clawSubsystem::firmlyGripped),
                        new ConditionalCommand(
                                new InstantCommand(() -> extendoSubsystem.setTargetPosition(400), extendoSubsystem),
                                new InstantCommand(),
                                () -> extendoSubsystem.getExtension() < 400
                        ),
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.LOW_BASKET)
                        ),
                        new WaitCommand(270),
                        new InstantCommand(() -> armSubsystem.setWristState(
                                ArmSubsystem.WristState.BASKET_OUTTAKE
                        )),
                        new WaitCommand(120),
                        new InstantCommand(() -> armSubsystem.setArmState(
                                ArmSubsystem.ArmState.BASKET_OUTTAKE
                        ))
                )
        );

        new Trigger(() -> driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0)
            .whenActive(
                new SequentialCommandGroup(
                    new InstantCommand(() -> extendoSubsystem.set_MAX_POWER(1)),
                    new InstantCommand(() -> extendoSubsystem.setTargetPosition((int)(1700 * driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)))
                )
            )
        );

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(intakeSubsystem::lower),
                    intake_sample()
                )
            );

        // High Basket
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(clawSubsystem::firmlyGripped),
                        new ConditionalCommand(
                                new InstantCommand(() -> extendoSubsystem.setTargetPosition(400), extendoSubsystem),
                                new InstantCommand(),
                                () -> extendoSubsystem.getExtension() < 400
                        ),
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.HIGH_BASKET)
                        ),
                        new WaitCommand(270),
                        new InstantCommand(() -> armSubsystem.setWristState(
                                ArmSubsystem.WristState.BASKET_OUTTAKE
                        )),
                        new WaitCommand(120),
                        new InstantCommand(() -> armSubsystem.setArmState(
                                ArmSubsystem.ArmState.BASKET_OUTTAKE
                        ))
                )
            )
            .whenReleased(
                new SequentialCommandGroup(
                    new InstantCommand(clawSubsystem::release),
                    new WaitCommand(150),
                    new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                    new InstantCommand(
                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                    ),
                    new ConditionalCommand(
                        new InstantCommand(() -> extendoSubsystem.setTargetPosition(250), extendoSubsystem),
                        new InstantCommand(),
                        () -> extendoSubsystem.getExtension() < 250
                    ),
                    new InstantCommand(() -> armSubsystem.setWristState(
                        ArmSubsystem.WristState.INTAKE
                    )),
                    new InstantCommand(() -> armSubsystem.setArmState(
                        ArmSubsystem.ArmState.INTAKE
                    ))
                )
            );

        //// Basket Outtake Automation
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.4)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(clawSubsystem::release),
                        new WaitCommand(1200),
                        new InstantCommand(() -> armSubsystem.setWristState(
                                ArmSubsystem.WristState.INTAKE
                        )),
                        new InstantCommand(clawSubsystem::justOpen),
                        new WaitCommand(120),
                        new InstantCommand(
                                () -> armSubsystem.setArmState(ArmSubsystem.ArmState.INTAKE)
                        ),
                        new InstantCommand(
                                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                        ),
                        new ConditionalCommand(
                                new InstantCommand(() -> extendoSubsystem.setTargetPosition(200), extendoSubsystem),
                                new InstantCommand(),
                                () -> extendoSubsystem.getExtension() < 200
                        )
                ));

        //// Chamber Outtake Automation
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.4) // PEOS SCORE
                .whenActive(new SequentialCommandGroup(
                    new InstantCommand(() -> armSubsystem.setArmState(
                        ArmSubsystem.ArmState.SPEC_OUTTAKE_NEW
                    )),
                    new InstantCommand(() -> armSubsystem.setWristState(
                        ArmSubsystem.WristState.SPEC_OUTTAKE_NEW
                    )),
                    new WaitCommand(300),
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
                        new InstantCommand(clawSubsystem::firmlyGripped, clawSubsystem),
                        new InstantCommand(extendoSubsystem::returnToZero, extendoSubsystem),
                        new InstantCommand(() -> armSubsystem.setWristState(
                                ArmSubsystem.WristState.PARK
                        )),
                        new WaitCommand(120),
                        new InstantCommand(() -> armSubsystem.setArmState(
                                ArmSubsystem.ArmState.PARK
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
                                () -> elevatorSubsystem.getHeight() < 30
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

        driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new SequentialCommandGroup(
                new InstantCommand(elevatorSubsystem::reset_encoder, elevatorSubsystem),
                new InstantCommand(extendoSubsystem::reset_encoder, extendoSubsystem)
        ));

        driverOp.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(intakeSubsystem::wiper_full_open),
                    new WaitCommand(400),
                    new InstantCommand(intakeSubsystem::wiper_contract)
                )
            );

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .toggleWhenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(pedroSubsystem::initialize),
                    new InstantCommand(() -> drive.setInAutoMode(pedroSubsystem.getIsAutoRunning()))
                )
            );

        driverOp.getGamepadButton(GamepadKeys.Button.A)
            .toggleWhenPressed(
                new SequentialCommandGroup(
                    new InstantCommand(pedroSubsystem::stop),
                    new InstantCommand(() -> drive.setInAutoMode(pedroSubsystem.getIsAutoRunning()))
                )
            );
    }

    @Override
    public double drivetrainForward() {
        return super.drivetrainForward();
    }

    @Override
    public double drivetrainStrafe() {
//        if (strafeControllerSubsystem.isEnabled()) return strafeControllerSubsystem.calculatePower();

        return super.drivetrainStrafe();
    }

    @Override
    public double drivetrainTurn() {
        if (gyroFollow != null && gyroFollow.isEnabled()) return -gyroFollow.calculateTurn();

        return super.drivetrainTurn();
    }
}
