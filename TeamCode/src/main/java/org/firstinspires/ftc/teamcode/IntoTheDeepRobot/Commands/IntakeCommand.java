package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.ExtendoSubsystem;
import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;

import java.util.concurrent.TimeUnit;

public class IntakeCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final ExtendoSubsystem extendo;

    public enum COLOR {
        BLUE,
        RED,
        YELLOW,
        BLUE_YELLOW,
        RED_YELLOW
    }

    private final COLOR color_preference;
    private Timing.Timer timer;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, COLOR color_preference, long MAX_ms,
                         ExtendoSubsystem extendo) {
        this.intake = intakeSubsystem;
        this.color_preference = color_preference;
        this.timer = new Timing.Timer(MAX_ms, TimeUnit.MILLISECONDS);
        this.extendo = extendo;
        addRequirements(intakeSubsystem, extendo);
    }

    public IntakeCommand(IntakeSubsystem intakeSubsystem, COLOR color_preference, ExtendoSubsystem extendo) {
        this.intake = intakeSubsystem;
        this.color_preference = color_preference;
        this.timer = new Timing.Timer(125000, TimeUnit.MILLISECONDS);
        this.extendo = extendo;

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
            extendo.blockManual(false);
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
