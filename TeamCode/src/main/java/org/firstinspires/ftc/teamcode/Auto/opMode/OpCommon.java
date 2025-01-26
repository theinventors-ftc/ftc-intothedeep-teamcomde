package org.firstinspires.ftc.teamcode.Auto.opMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    protected HardwareMap hardwareMap;
    public ClawSubsystem clawSubsystem;
    public ArmSubsystem armSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public ExtendoSubsystem extendoSubsystem;
    public CouplersSubsystem couplersSubsystem;
    public DistanceSensorsSubsystem distanceSensorsSubsystem;

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

    public SequentialCommandGroup reset_elevator() {
        return new SequentialCommandGroup(
            new WaitCommand(2000),
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
}
