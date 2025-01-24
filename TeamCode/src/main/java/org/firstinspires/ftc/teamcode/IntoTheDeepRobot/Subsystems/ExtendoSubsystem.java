package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.controllers.PIDFControllerEx;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

@Config
public class ExtendoSubsystem extends SubsystemBase {
    private MotorExEx extendoMotor;
    private final double MAX_EXTENDO_POWER = 1.0;
    private final int MAX_EXTENSION = 0;
    private DoubleSupplier power;

    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
            0.0,
            1.0,
            0.0
    );
    private PIDFControllerEx pid = new PIDFControllerEx(
            0.007,
            0.08,
            0.06,
            0.0,
            0.0,
            0.0,
            80,
            0.6
    );

    public static int targetPosition = 0;
    private int spring_off = 25;
    private Telemetry telemetry;

    // Zeroing
    public boolean isStalled = false;
    public double ampThreshold = 3.4;
    private final Timing.Timer timer;
    private boolean found_zero = false;

    public ExtendoSubsystem(RobotMap robotMap, DoubleSupplier power, Telemetry telemetry) {
        extendoMotor = robotMap.getExtendoMotor();
//        extendoMotor.setInverted(true);
        extendoMotor.setRunMode(MotorExEx.RunMode.RawPower);
        extendoMotor.resetEncoder();
        this.power = power;

        this.telemetry = telemetry;

        timer = new Timing.Timer(600, TimeUnit.MILLISECONDS);
    }

    private void set(double power) {
        extendoMotor.set(ff.calculate(power * MAX_EXTENDO_POWER));
    }

    @Override
    public void periodic() {
        if (!found_zero) {
            searchZero();
            return;
        }

        pid.setSetPoint(targetPosition);

        telemetry.addData("Extendo Position", getExtension());
        telemetry.update();

        if(Math.abs(power.getAsDouble()) > 0.05){ // Manual
            set(power.getAsDouble());
            targetPosition = getExtension();
        } else { // Auto
            set(pid.calculate(getExtension()));
        }
    }

    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    public void returnToZero() {
        setTargetPosition(0);
    }

    public boolean atTarget() {
        return Math.abs(getExtension() - targetPosition) < 30;
    }

    public int getExtension() {
        return extendoMotor.getCurrentPosition()-spring_off;
    }

    // Zeroing
    public void searchZero() {
        if (!isExtendoBack()) {
            set(-0.6);

            if(getCurrent() > 4.5 && !timer.isTimerOn()) {
                timer.start();
            }

            if(timer.done() && getCurrent() > ampThreshold){
                isStalled = true;
            }
        } else {
            set(0);
            extendoMotor.resetEncoder();
            found_zero = true;
        }
    }

    public double getCurrent() {
        return ((DcMotorEx)extendoMotor.getRawMotor()).getCurrent(CurrentUnit.AMPS);
    }

    public boolean isExtendoBack() {
        return isStalled;
    }
}
