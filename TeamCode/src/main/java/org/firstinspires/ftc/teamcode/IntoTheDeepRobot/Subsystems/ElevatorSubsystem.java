package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
    private MotorExEx elevatorMotor;
    private final double MAX_ELEVATOR_POWER = 1.0;
    private DoubleSupplier power;

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
    public ElevatorSubsystem(RobotMap robotMap, DoubleSupplier power) {
        elevatorMotor = robotMap.getSliderMotor();
        elevatorMotor.setRunMode(MotorExEx.RunMode.RawPower);
        this.power = power;
    }

    @Override
    public void periodic() {
        elevatorMotor.set(power.getAsDouble() * MAX_ELEVATOR_POWER);
    }
}
