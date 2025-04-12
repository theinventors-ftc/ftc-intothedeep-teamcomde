package org.firstinspires.ftc.teamcode.Auto.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.ForwardControllerSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.HeadingControllerSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Controllers.StrafeControllerSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.CouplersSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.DistanceSensorsSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.RobotEx;

public class OpCommon {
    private RobotMap robotMap;
    private FtcDashboard dashboard;
    private RobotEx.Alliance alliance;

    /*--Subsystems--*/
//    protected HardwareMap hardwareMap;
    public ClawSubsystem clawSubsystem;
    public ArmSubsystem armSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public CouplersSubsystem couplersSubsystem;
    public DistanceSensorsSubsystem distanceSensorsSubsystem;

    /*--Controllers--*/
    public ForwardControllerSubsystem forwardControllerSubsystem;
    public StrafeControllerSubsystem strafeControllerSubsystem;
    public HeadingControllerSubsystem gyroFollow;

    /**
     * Initialization of all subsystems and mechanisms
     */
    public OpCommon(RobotMap robotMap, RobotEx.Alliance alliance) {
        this.alliance = alliance;
        this.robotMap = robotMap;
        clawSubsystem = new ClawSubsystem(this.robotMap);
        armSubsystem = new ArmSubsystem(this.robotMap);
        intakeSubsystem = new IntakeSubsystem(this.robotMap);
        elevatorSubsystem = new ElevatorSubsystem(
            this.robotMap,
            () -> 0.0,
            robotMap.getRearLeftMotor(),
            robotMap.getTelemetry(),
            false
        );
        extendoSubsystem = new ExtendoSubsystem(
            this.robotMap,
            () -> 0.0,
            robotMap.getTelemetry(),
            false
        );
        couplersSubsystem = new CouplersSubsystem(this.robotMap);
        distanceSensorsSubsystem = new DistanceSensorsSubsystem(
            this.robotMap,
            robotMap.getTelemetry()
        );
    }

    public void init_controllers(Follower drive) {
        this.dashboard = FtcDashboard.getInstance();
        forwardControllerSubsystem = new ForwardControllerSubsystem(
            () -> distanceSensorsSubsystem.getDistances()[0],
            dashboard.getTelemetry()
        );
        strafeControllerSubsystem = new StrafeControllerSubsystem(
            () -> distanceSensorsSubsystem.getDistances()[2], // TODO Change to 2
            dashboard.getTelemetry()
        );
        gyroFollow = new HeadingControllerSubsystem(
            () -> Math.toDegrees(drive.getPose().getHeading()),
            () -> 0,
            dashboard.getTelemetry()
        );
    }

    public SequentialCommandGroup reset_elevator() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> armSubsystem.setWristState(
                ArmSubsystem.WristState.PARK
            )),
            new WaitCommand(120),
            new InstantCommand(
                () -> armSubsystem.setArmState(ArmSubsystem.ArmState.PARK)
            ),
            new InstantCommand(
                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
            )
        );
    }

    public SequentialCommandGroup deep_intake() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> extendoSubsystem.setTargetPosition(500), extendoSubsystem),
                new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
                new InstantCommand(intakeSubsystem::lower),
                new InstantCommand(
                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                ),
                new InstantCommand(() -> armSubsystem.setWristState(
                        ArmSubsystem.WristState.INTAKE
                )),
                new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                new WaitCommand(120),
                new InstantCommand(() -> armSubsystem.setArmState(
                        ArmSubsystem.ArmState.INTAKE
                )),
                // Clear Way
                new InstantCommand(() -> extendoSubsystem.set_MAX_POWER(0.8)),
                new InstantCommand(() -> extendoSubsystem.setTargetPosition(850)),
                new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
                new InstantCommand(() -> extendoSubsystem.set_MAX_POWER(1)),
                new InstantCommand(() -> extendoSubsystem.setTargetPosition(480)),
                new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
                new InstantCommand(() -> extendoSubsystem.set_MAX_POWER(0.8)),
                new InstantCommand(() -> extendoSubsystem.setTargetPosition(650)),
                new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
                new InstantCommand(() -> extendoSubsystem.set_MAX_POWER(1)),
                new InstantCommand(() -> extendoSubsystem.setTargetPosition(480)),
                new WaitUntilCommand(() -> extendoSubsystem.atTarget())
        );
    }

    public SequentialCommandGroup submersible_intake() {
        return new SequentialCommandGroup(
                // Go to Park State: Elevator, Arm, Wrist, Claw
                new InstantCommand(()-> extendoSubsystem.set_MAX_POWER(0.4)),
                new InstantCommand(()-> extendoSubsystem.setTargetPosition(1600)),
                // Intake Procedure
                new IntakeCommand(
                        intakeSubsystem,
                        (alliance == RobotEx.Alliance.RED ?
                                IntakeCommand.COLOR.RED_YELLOW : IntakeCommand.COLOR.BLUE_YELLOW),
                        extendoSubsystem
                ),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(
                                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                                ),
                                new InstantCommand(() -> extendoSubsystem.set_MAX_POWER(1)),
                                new InstantCommand(() -> extendoSubsystem.setTargetPosition(90), extendoSubsystem),
                                // Push Sample (Align to Parrot)
                                new InstantCommand(intakeSubsystem::run),
                                new WaitCommand(120),
                                new InstantCommand(intakeSubsystem::stop),
                                new InstantCommand(intakeSubsystem::raise, intakeSubsystem),
                                new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
                                new WaitUntilCommand(() -> elevatorSubsystem.atTarget()),
                                new InstantCommand(clawSubsystem::looslyGripped),
                                new WaitCommand(60)
                        ),
                        discard_sample(),
                        () -> intakeSubsystem.check_color(alliance, true)
                )
        );
    }

    public SequentialCommandGroup sample_intake() {
        return new SequentialCommandGroup(
            // Go to Park State: Elevator, Arm, Wrist, Claw
            new InstantCommand(intakeSubsystem::lower),
            new InstantCommand(
                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
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
                    (alliance == RobotEx.Alliance.RED ?
                        IntakeCommand.COLOR.RED_YELLOW : IntakeCommand.COLOR.BLUE_YELLOW),
                        extendoSubsystem,
                        0.7,
                        2500
                )
            ),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(
                        () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
                    ),
                    new InstantCommand(() -> extendoSubsystem.set_MAX_POWER(1)),
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
                    )
                ),
                discard_sample(),
                () -> intakeSubsystem.check_color(alliance, true)
            )
        );
    }

    public SequentialCommandGroup discard_sample() {
        return new SequentialCommandGroup(
            new InstantCommand(intakeSubsystem::raise),
            new InstantCommand(intakeSubsystem::reverse),
            new WaitCommand(1000),
            new InstantCommand(intakeSubsystem::stop)
        );
    }

    public SequentialCommandGroup basket_scoring() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.HIGH_BASKET)
            ),
            new InstantCommand(clawSubsystem::firmlyGripped, clawSubsystem),
            new WaitCommand(100),
            new InstantCommand(() -> armSubsystem.setArmState(
                ArmSubsystem.ArmState.BASKET_OUTTAKE
            )),
            new WaitCommand(50),
            new WaitUntilCommand(() -> elevatorSubsystem.atTarget()),
            new InstantCommand(() -> armSubsystem.setWristState(
                ArmSubsystem.WristState.BASKET_OUTTAKE
            ))
        );
    }

    public SequentialCommandGroup release_sample() {
        return new SequentialCommandGroup(
            new InstantCommand(clawSubsystem::release),
            new WaitCommand(250),
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
        );
    }

    public SequentialCommandGroup extendo(double power) {
        return new SequentialCommandGroup(
            new InstantCommand(intakeSubsystem::lower),
            new InstantCommand(()-> extendoSubsystem.set_MAX_POWER(power)),
            new InstantCommand(()-> extendoSubsystem.setTargetPosition(1200))
        );
    }

    public SequentialCommandGroup extendo3dSample(double power) {
        return new SequentialCommandGroup(
            new InstantCommand(intakeSubsystem::lower),
            new InstantCommand(()-> extendoSubsystem.set_MAX_POWER(power)),
            new InstantCommand(()-> extendoSubsystem.setTargetPosition(800))
        );
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

    public SequentialCommandGroup extendoSpecimenPush() {
        return new SequentialCommandGroup(
            new IntakeCommand(
                intakeSubsystem,
                  (alliance == RobotEx.Alliance.RED ?
                      IntakeCommand.COLOR.RED_YELLOW : IntakeCommand.COLOR.BLUE_YELLOW),
                  extendoSubsystem,
                  0.7,
                  1000
            ),
            new InstantCommand(()-> extendoSubsystem.set_MAX_POWER(1)),
            new InstantCommand(()-> extendoSubsystem.setTargetPosition(900)),
            new WaitCommand(100),
            new InstantCommand(intakeSubsystem::reverse),
            new WaitUntilCommand(extendoSubsystem::atTarget),
            new InstantCommand(()-> extendoSubsystem.set_MAX_POWER(1)),
            new InstantCommand(()-> extendoSubsystem.setTargetPosition(150)),
            new InstantCommand(intakeSubsystem::stop)
        );
    }

    public SequentialCommandGroup armReset() {
        return new SequentialCommandGroup(
            new InstantCommand(clawSubsystem::firmlyGripped, clawSubsystem),
            new InstantCommand(() -> armSubsystem.setWristState(
                ArmSubsystem.WristState.PARK
            )),
            new WaitCommand(120),
            new InstantCommand(() -> armSubsystem.setArmState(
                ArmSubsystem.ArmState.PARK
            ))
        );
    }

    public SequentialCommandGroup parking() {
        return new SequentialCommandGroup(
                new InstantCommand(()-> armSubsystem.setArmState(ArmSubsystem.ArmState.LVL1_ASCENT)),
                new InstantCommand(()-> armSubsystem.setWristState(ArmSubsystem.WristState.LVL1_ASCENT))
        );
    }
}
