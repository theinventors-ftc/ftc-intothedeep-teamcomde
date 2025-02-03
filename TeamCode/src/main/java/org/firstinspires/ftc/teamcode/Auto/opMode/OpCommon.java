package org.firstinspires.ftc.teamcode.Auto.opMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.drive.SampleMecanumDrive;
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
import org.inventors.ftc.robotbase.hardware.GamepadExEx;

public class OpCommon {

    private RobotMap robotMap;
    private FtcDashboard dashboard;

    /*--Subsystems--*/
    protected HardwareMap hardwareMap;
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
    public OpCommon(RobotMap robotMap) {
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

        //Auto release speciment
        new Trigger(
            () -> distanceSensorsSubsystem.getDistances()[0] <=
                (armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_LOW ? 6.7 : 14.9) &&
                (armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_LOW ||
                    armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_HIGH)
        ).whenActive(new ConditionalCommand(
            new SequentialCommandGroup(
                new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                new WaitCommand(150),
                new InstantCommand(
                    () -> armSubsystem.setArmState(ArmSubsystem.ArmState.PERP) //TODO
                )
            ),
            new SequentialCommandGroup(
                new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
                new WaitCommand(150),
                new InstantCommand(
                    () -> armSubsystem.setArmState(ArmSubsystem.ArmState.INTAKE_B) //TODO
                )
            ),
            () -> armSubsystem.getArmState() == ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_HIGH
        ));
    }

    public void init_controllers(SampleMecanumDrive drive) {
        this.dashboard = FtcDashboard.getInstance();
        forwardControllerSubsystem = new ForwardControllerSubsystem(
            () -> distanceSensorsSubsystem.getDistances()[0],
            dashboard.getTelemetry()
        );
        strafeControllerSubsystem = new StrafeControllerSubsystem(
            () -> distanceSensorsSubsystem.getDistances()[1], // TODO Change to 2
            dashboard.getTelemetry()
        );
        gyroFollow = new HeadingControllerSubsystem(
            () -> Math.toDegrees(drive.getPoseEstimate().getHeading()),
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

    public SequentialCommandGroup raise_high_chamber() {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.HIGH_CHAMBER)
            ),
            new WaitCommand(50),
            new InstantCommand(() -> armSubsystem.setWristState(
                ArmSubsystem.WristState.SPECIMENT_OUTTAKE_HIGH
            )),
            new WaitCommand(120),
            new InstantCommand(() -> armSubsystem.setArmState(
                ArmSubsystem.ArmState.SPECIMENT_OUTTAKE_HIGH
            )),
            new WaitCommand(5000)
        );
    }

    public SequentialCommandGroup sample_intake() {
        return new SequentialCommandGroup(
            // Lower Intake Arm System
            new InstantCommand(intakeSubsystem::lower),
            new WaitCommand(80),
            // Go to Park State: Elevator, Arm, Wrist, Claw
            new InstantCommand(
                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.PARK)
            ),
            new InstantCommand(() -> armSubsystem.setWristState(
                ArmSubsystem.WristState.INTAKE
            )),
            new InstantCommand(clawSubsystem::goNormal, clawSubsystem),
            new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
            new WaitCommand(120),
            new InstantCommand(() -> armSubsystem.setArmState(
                ArmSubsystem.ArmState.INTAKE
            )),
            // Intake Procedure
            new IntakeCommand(
                intakeSubsystem,
                IntakeCommand.COLOR.RED_YELLOW
            ),
            new InstantCommand(intakeSubsystem::brake_reverse),
            new WaitCommand(80),
            new InstantCommand(intakeSubsystem::stop),
            new WaitCommand(50),
            // Raise Intake and Return Extendo
            new InstantCommand(intakeSubsystem::raise, intakeSubsystem),
            // FAILSAFE
            new InstantCommand(() -> armSubsystem.setWristState(
                ArmSubsystem.WristState.INTAKE
            )),
            new InstantCommand(clawSubsystem::goNormal, clawSubsystem),
            new InstantCommand(clawSubsystem::justOpen, clawSubsystem),
            new WaitCommand(120),
            new InstantCommand(() -> armSubsystem.setArmState(
                ArmSubsystem.ArmState.INTAKE
            )),
            //
            new InstantCommand(()-> extendoSubsystem.set_MAX_POWER(1)),
            new InstantCommand(extendoSubsystem::returnToZero, extendoSubsystem),
            new WaitUntilCommand(() -> extendoSubsystem.atTarget()),
            // Push Sample (Align to Parrot)
            new InstantCommand(intakeSubsystem::run),
            new WaitCommand(240),
            new InstantCommand(intakeSubsystem::stop),
            new WaitCommand(50),
            // Lower Slider w/ Claw and grab Sample
            new InstantCommand(
                () -> elevatorSubsystem.setLevel(ElevatorSubsystem.Level.INTAKE)
            ),
            new WaitUntilCommand(()-> elevatorSubsystem.atTarget()),
            new InstantCommand(clawSubsystem::grab),
            new WaitCommand(200)
        );
    }

    public SequentialCommandGroup basket_scoring() {
        return new SequentialCommandGroup(
            // Disengage Sample from the Intake/Parrot
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
            )),
            new WaitUntilCommand(elevatorSubsystem::atTarget)
        );
    }

    public SequentialCommandGroup release_sample() {
        return new SequentialCommandGroup(
            new WaitCommand(250),
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
        );
    }

    public SequentialCommandGroup extendo() {
        return new SequentialCommandGroup(
            new InstantCommand(()-> extendoSubsystem.set_MAX_POWER(0.5)),
            new InstantCommand(()-> extendoSubsystem.setTargetPosition(1065))
        );
    }

    public SequentialCommandGroup activateDistanceCalibration() {
        return new SequentialCommandGroup(
            new InstantCommand(gyroFollow::enable),
            new InstantCommand(strafeControllerSubsystem::enable),
            new InstantCommand(forwardControllerSubsystem::enable),
            new InstantCommand(() -> gyroFollow.setGyroTarget(75)),
            new InstantCommand(() -> strafeControllerSubsystem.setDistTarget(3.555)),
            new InstantCommand(() -> forwardControllerSubsystem.setGyroTarget(44.615))
        );
    }

    public SequentialCommandGroup deactivateDistanceCalibration() {
        return new SequentialCommandGroup(
            new InstantCommand(gyroFollow::disable),
            new InstantCommand(strafeControllerSubsystem::disable),
            new InstantCommand(forwardControllerSubsystem::disable)
        );
    }
}
