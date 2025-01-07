package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.util;

import org.inventors.ftc.robotbase.hardware.MotorExEx;

public class MotorPair {
    private final MotorExEx leadMotor;
    private final MotorExEx followMotor;
    private double speedRatio;

    public MotorPair(MotorExEx leadMotor, MotorExEx followMotor, double speedRatio) {
        this.leadMotor = leadMotor;
        this.followMotor = followMotor;
        this.speedRatio = speedRatio;
    }

    public void set(double power) {
        leadMotor.set(power);
        followMotor.set(power * speedRatio);
    }

    public void lead_setInverted(boolean inverted) {
        leadMotor.setInverted(inverted);
    }

    public void follow_setInverted(boolean inverted) {
        followMotor.setInverted(inverted);
    }

    public void setSpeedRatio(double speedRatio) {
        this.speedRatio = speedRatio;
    }

    public double getSpeedRatio() {
        return speedRatio;
    }
}
