package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;

import java.util.concurrent.TimeUnit;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;

    public enum COLOR {
        BLUE,
        RED,
        YELLOW,
        BLUE_YELLOW,
        RED_YELLOW
    }

    private final COLOR color_preference;
    private Timing.Timer timer;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, COLOR color_preference, long MAX_ms) {
        this.intake = intakeSubsystem;
        this.color_preference = color_preference;
        this.timer = new Timing.Timer(MAX_ms, TimeUnit.MILLISECONDS);
        addRequirements(intakeSubsystem);
    }

    public IntakeCommand(IntakeSubsystem intakeSubsystem, COLOR color_preference) {
        this.intake = intakeSubsystem;
        this.color_preference = color_preference;
        this.timer = new Timing.Timer(125000, TimeUnit.MILLISECONDS);
        addRequirements(intakeSubsystem);
    }

    public boolean check_color(IntakeSubsystem.COLOR sample_color) {;
        switch (color_preference) {
            case BLUE:
                return sample_color == IntakeSubsystem.COLOR.BLUE;
            case RED:
                return sample_color == IntakeSubsystem.COLOR.RED;
            case YELLOW:
                return sample_color == IntakeSubsystem.COLOR.YELLOW;
            case BLUE_YELLOW:
                return sample_color == IntakeSubsystem.COLOR.BLUE || sample_color == IntakeSubsystem.COLOR.YELLOW;
            case RED_YELLOW:
                return sample_color == IntakeSubsystem.COLOR.RED || sample_color == IntakeSubsystem.COLOR.YELLOW;
            default:
                return false;
        }
    }

    public void initialize() {
        timer.start();
        intake.run();
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            intake.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return (intake.isSample()
//                &&
//                check_color(intake.getSampleColor())
                &&
                intake.getSampleColor() != IntakeSubsystem.COLOR.NONE)
                || timer.done();
    }
}
