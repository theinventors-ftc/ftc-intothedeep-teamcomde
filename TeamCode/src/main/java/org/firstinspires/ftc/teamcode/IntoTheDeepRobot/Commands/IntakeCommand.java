package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems.IntakeSubsystem;

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

    public IntakeCommand(IntakeSubsystem intakeSubsystem, COLOR color_preference) {
        this.intake = intakeSubsystem;
        this.color_preference = color_preference;
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
        return intake.isSample()
                &&
                check_color(intake.getSampleColor())
                &&
                intake.getSampleColor() != IntakeSubsystem.COLOR.NONE;
    }
}
