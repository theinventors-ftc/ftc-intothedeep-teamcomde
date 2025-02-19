package org.firstinspires.ftc.teamcode.Auto.opMode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

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

    public void init_controllers(SampleMecanumDrive drive) {
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
            () -> Math.toDegrees(drive.getPoseEstimate().getHeading()),
            () -> 0,
            dashboard.getTelemetry()
        );
    }

    public double calculate_turn() {
        return Range.clip(-gyroFollow.calculateTurn(), -1, 1);
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
                    (alliance == RobotEx.Alliance.RED ?
                        IntakeCommand.COLOR.RED_YELLOW : IntakeCommand.COLOR.BLUE_YELLOW)
                )
            ),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> extendoSubsystem.blockManual(true)),
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
                    new InstantCommand(clawSubsystem::grab),
                    new WaitCommand(60),
                    new InstantCommand(() -> extendoSubsystem.blockManual(false))
                ),
                discard_sample(),
                () -> intakeSubsystem.check_color(alliance)
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

    public SequentialCommandGroup basket_scoring() {
        return new SequentialCommandGroup(
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
            new WaitUntilCommand(() -> elevatorSubsystem.atTarget())
        );
    }

    public SequentialCommandGroup release_sample() {
        return new SequentialCommandGroup(
            new WaitCommand(250),
            new InstantCommand(clawSubsystem::release),
            new WaitCommand(400),
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
            new InstantCommand(()-> extendoSubsystem.setTargetPosition(700))
        );
    }

    public SequentialCommandGroup activateDistanceCalibration(double[] pidaTargets) {
        return new SequentialCommandGroup(
            new InstantCommand(gyroFollow::enable),
            new InstantCommand(strafeControllerSubsystem::enable),
            new InstantCommand(forwardControllerSubsystem::enable),
            new InstantCommand(() -> gyroFollow.setGyroTarget(pidaTargets[2])),
            new InstantCommand(() -> strafeControllerSubsystem.setDistTarget(pidaTargets[1])),
            new InstantCommand(() -> forwardControllerSubsystem.setGyroTarget(pidaTargets[0]))
        );
    }

    public SequentialCommandGroup deactivateDistanceCalibration() {
        return new SequentialCommandGroup(
            new InstantCommand(gyroFollow::disable),
            new InstantCommand(strafeControllerSubsystem::disable),
            new InstantCommand(forwardControllerSubsystem::disable)
        );
    }

    public SequentialCommandGroup specimenAim() {
        return new SequentialCommandGroup(
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
        );
    }

    public SequentialCommandGroup specimenIntake() {
        return new SequentialCommandGroup(
            new InstantCommand(clawSubsystem::grab),
            new WaitCommand(200),
            new InstantCommand(() -> elevatorSubsystem.setLevel(
                ElevatorSubsystem.Level.HIGH_CHAMBER
            )),
            new WaitCommand(150),
            new InstantCommand(() -> armSubsystem.setWristState(
                ArmSubsystem.WristState.SPECIMEN_OUTTAKE
            )),
            new WaitCommand(100),
            new InstantCommand(clawSubsystem::goNormal),
            new InstantCommand(() -> armSubsystem.setArmState(
                ArmSubsystem.ArmState.SPECIMEN_OUTTAKE
            ))
        );
    }

    public SequentialCommandGroup scoreSpeciment() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> elevatorSubsystem.setLevel(
                ElevatorSubsystem.Level.HIGH_CHAMBER_RELEASE
            )),
            new WaitUntilCommand(() -> elevatorSubsystem.atTarget())
        );
    }

    public SequentialCommandGroup releaseSpecimnt() {
        return new SequentialCommandGroup(
            new InstantCommand(clawSubsystem::release),
            new WaitCommand(100)
        );
    }

    public double drivetrainStrafe() {
        return strafeControllerSubsystem.calculatePower();
    }

    public double drivetrainForward() {
        return forwardControllerSubsystem.calculatePower();
    }

    public double drivetrainTurn() {
        return -gyroFollow.calculateTurn();
    }

    public void robotCentricMovement(double x, double y, double t) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(y) + abs(x) + abs(t), 1);
        double frontLeftPower = (y + x + t) / denominator;
        double backLeftPower = (y - x + t) / denominator;
        double frontRightPower = (y - x - t) / denominator;
        double backRightPower = (y + x - t) / denominator;

        robotMap.getFrontLeftMotor().set(frontLeftPower);
        robotMap.getRearLeftMotor().set(backLeftPower);
        robotMap.getFrontRightMotor().set(frontRightPower);
        robotMap.getRearRightMotor().set(backRightPower);
    }
}
