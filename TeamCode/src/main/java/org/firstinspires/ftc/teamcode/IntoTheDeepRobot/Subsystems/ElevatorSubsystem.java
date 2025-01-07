package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

public class ElevatorSubsystem extends SubsystemBase {
    private MotorExEx elevatorMotor;
    private final double MAX_ELEVATOR_POWER = 1.0;

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(
            0.0,
            0.0,
            1.0,
            0.0
    );
    private PIDFControllerEx pid = new PIDFControllerEx(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
    );
    public ElevatorSubsystem(RobotMap robotMap) {
        elevatorMotor = robotMap.getSliderMotor();

    }
}
