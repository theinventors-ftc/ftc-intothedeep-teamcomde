package org.firstinspires.ftc.teamcode.IntoTheDeepRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Pipelines.SpecimenDetectionPipeline;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.CouplersSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.DistanceSensorsSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.HangingSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.HeadingControllerSubsystem;

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
    protected Limelight3A limelight3A;

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

    public SequentialCommandGroup intake_sample_for_specimen() {
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
                                new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
                                new WaitUntilCommand(() -> elevatorSubsystem.atTarget()),
                                new WaitCommand(30),
                                new InstantCommand(clawSubsystem::looslyGripped),
                                new WaitCommand(60),
                                // Disengage Sample from the Intake/Parrot
                                new InstantCommand(
                                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.PARK2)
                                ),
                                new InstantCommand(() -> extendoSubsystem.blockManual(false)),
                                new WaitCommand(400),
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

        //------------------------------------ Manual Actions ---------------------------------- //
//         Claw Grab/Release(Just Open) Toggle
        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ConditionalCommand(
                new InstantCommand(clawSubsystem::firmlyGripped),
                new InstantCommand(clawSubsystem::justOpen),
                () -> clawSubsystem.getState() != ClawSubsystem.ClawState.FIRMLY_GRIPPED
        ));

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

        // High Basket
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new SequentialCommandGroup(
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

        toolOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new SequentialCommandGroup(
                new InstantCommand(elevatorSubsystem::reset_encoder, elevatorSubsystem),
                new InstantCommand(extendoSubsystem::reset_encoder, extendoSubsystem)
        ));

        // Specimen Outtake Automation with Distance Sensor (6.3, 13)
//        new Trigger(
//                () -> distanceSensorsSubsystem.getDistances()[0] <=
//                        (armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_LOW ? 6.7 : 13) &&
//                        (armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_LOW ||
//                                armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_HIGH)
//        ).whenActive(new ConditionalCommand(
//                new SequentialCommandGroup(
//                        new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
//                        new WaitCommand(150),
//                        new InstantCommand(
//                                () -> armSubsystem.setArmState(ArmSubsystem.ArmState.PERP)
//                        )
//                ),
//                new SequentialCommandGroup(
//                        new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
//                        new WaitCommand(150),
//                        new InstantCommand(
//                                () -> armSubsystem.setArmState(ArmSubsystem.ArmState.INTAKE_B)
//                        )
//                ),
//                () -> armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_HIGH
//        ));

        // ------------------------------------ Drive Commands ---------------------------------- //
        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(gyroFollow::enable),
                                new InstantCommand(() -> gyroFollow.setGyroTarget(0))
                        ),
                        new InstantCommand(gyroFollow::disable)
                );

        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(
                new ConditionalCommand(
                    new InstantCommand(intakeSubsystem::semi_open),
                    new InstantCommand(intakeSubsystem::contract),
                    () -> intakeSubsystem.getWiperState() != IntakeSubsystem.WiperState.SEMI_OPEN
                )
            );

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(
                new ConditionalCommand(
                    new InstantCommand(intakeSubsystem::full_open),
                    new InstantCommand(intakeSubsystem::contract),
                    () -> intakeSubsystem.getWiperState() != IntakeSubsystem.WiperState.FULL_OPEN
                )
            );

//        driverOp.getGamepadButton(GamepadKeys.Button.B)
//                .whenPressed(
//                        new SequentialCommandGroup(
//                                new InstantCommand(this::setRobotCentric),
//                                new InstantCommand(gyroFollow::enable),
//                                new InstantCommand(strafeControllerSubsystem::enable),
//                                new InstantCommand(() -> gyroFollow.setGyroTarget(0)),
//                                new InstantCommand(() -> strafeControllerSubsystem.setDistTarget(39.7))
//                        )
//                ).whenReleased(new SequentialCommandGroup(
//                        new InstantCommand(this::setFieldCentric),
//                        new InstantCommand(gyroFollow::disable),
//                        new InstantCommand(strafeControllerSubsystem::disable)
//                ));

//        toolOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(new PerpetualCommand(
//                new SequentialCommandGroup(
//                        new InstantCommand(elevatorSubsystem::lower, elevatorSubsystem),
//                        new InstantCommand(elevatorSubsystem::reset_encoder)
//                )
//        )).whenReleased(new InstantCommand(elevatorSubsystem::stop));
    }

    @Override
    public double drivetrainForward() {
//        return strafeControllerSubsystem.calculatePower();

        return super.drivetrainForward();
    }

    @Override
    public double drivetrainStrafe() {
//        if (strafeControllerSubsystem.isEnabled()) return strafeControllerSubsystem.calculatePower();
//        return specimenAlignmentSubsystem.calculatePower();

        return super.drivetrainStrafe();
    }

    @Override
    public double drivetrainTurn() {
        if (gyroFollow != null && gyroFollow.isEnabled()) return -gyroFollow.calculateTurn();

        return super.drivetrainTurn();
    }
}
