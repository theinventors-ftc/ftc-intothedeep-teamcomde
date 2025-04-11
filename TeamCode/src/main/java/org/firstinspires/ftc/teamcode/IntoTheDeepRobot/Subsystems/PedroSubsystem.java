package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.Tile;
import static org.firstinspires.ftc.teamcode.Auto.features.BuilderFunctions.robotY;

import android.se.omapi.Session;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.IntoTheDeepRobot.IntoTheDeepRobot;

public class PedroSubsystem extends SubsystemBase {

    private Follower follower;
    private boolean isAutoRunning = false;
    private SequentialCommandGroup scoreSpeciment;
    private SequentialCommandGroup releaseSpecimen;
    private SequentialCommandGroup specimenAim;

    private double
        xThreshold = 4,
        yThreshold = 4;

    private enum PathState {
        STARTING,
        CHAMBER_OBSERVATION,
        OBSERVATION_CHAMBER
    }

    private PathState pathState;

    private Pose
        chambers = new Pose(
        5, -Tile - (robotY/2) + 1,
        Math.toRadians(270), false),

    observationZone = new Pose(
        1.5 * Tile, -3 * Tile + (robotY/2),
        Math.toRadians(280), false);

    private Path
        toChambers,
        toObservationZone,
        firstCycle;

    public void buildFirstCycle() {
        firstCycle = new Path(
            new BezierCurve(
                new Point(new Pose(0, chambers.getY(), chambers.getHeading())),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(new Pose(1.5 * Tile, -2 * Tile, false)),
                new Point(observationZone)
            ));
        firstCycle.setLinearHeadingInterpolation(chambers.getHeading(), observationZone.getHeading());
    }

    public void buildChambers() {
        toChambers = new Path(
            new BezierCurve(
                new Point(observationZone),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(chambers)
            ));
        toChambers.setLinearHeadingInterpolation(observationZone.getHeading(), chambers.getHeading());
    }

    public void buildObservationZone() {
        toObservationZone = new Path(
            new BezierCurve(
                new Point(chambers),
                new Point(new Pose(0, -2 * Tile, false)),
                new Point(new Pose(1.5 * Tile, -2 * Tile, false)),
                new Point(observationZone)
            ));
        toObservationZone.setLinearHeadingInterpolation(chambers.getHeading(), observationZone.getHeading());
    }

    public void setPathState(PathState pState) {
        pathState = pState;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case STARTING:
                releaseSpecimen.schedule();
                specimenAim.schedule();
                buildChambers();

                follower.followPath(firstCycle, true);
                setPathState(PathState.OBSERVATION_CHAMBER);
                break;

            case OBSERVATION_CHAMBER:
                if(follower.atPose(observationZone, xThreshold, yThreshold)) {
                    scoreSpeciment.schedule();
                    buildObservationZone();

                    follower.followPath(toChambers, true);
                    setPathState(PathState.CHAMBER_OBSERVATION);
                }
                break;

            case CHAMBER_OBSERVATION:
                if (follower.atPose(chambers, xThreshold, yThreshold)) {
                    releaseSpecimen.schedule();
                    specimenAim.schedule();

                    chambers.setY(chambers.getY() + 2);
                    buildChambers();

                    follower.followPath(toObservationZone, true);
                    setPathState(PathState.OBSERVATION_CHAMBER);
                }
                break;
        }
    }

    public PedroSubsystem (Follower follower, SequentialCommandGroup scoreSpeciment,
                                                SequentialCommandGroup releaseSpecimen,
                                                SequentialCommandGroup specimenAim)
    {
        this.follower = follower;
        this.scoreSpeciment = scoreSpeciment;
        this.releaseSpecimen = releaseSpecimen;
        this.specimenAim = specimenAim;
    }

    public void setIsAutoRunning(boolean set) {
        isAutoRunning = set;
    }

    public boolean getIsAutoRunning() {
        return isAutoRunning;
    }

    public void initialize() {
        setIsAutoRunning(true);
        buildFirstCycle();
        setPathState(PathState.STARTING);
    }

    public void periodic() {
        if (getIsAutoRunning()) {
            follower.update();
            autonomousPathUpdate();
        }
    }

    public void stop() {
        follower.breakFollowing();
        setIsAutoRunning(false);
    }
}
