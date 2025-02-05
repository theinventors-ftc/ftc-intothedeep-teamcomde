package org.firstinspires.ftc.teamcode.Auto.features;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class BuilderFunctions {

    // TODO: Use this as an base for a way point so that its faster and easier
    /*
        trajectory = poseAdjuster(new Pose2d(
            -2 * Tile, 2 * Tile, Math.toRadians(45)
        ), BuilderFunctions.RobotSides.CENTER);
     */

    public static double
        robotY = 15.1875,
        robotX = 14;

    public static double
        Tile = 24; /*-inches-*/

    public enum RobotSides {FRONT, REAR, CENTER, LEFT, RIGHT}

    /**
     * Adjusts the target pose in order for a certain side of the robot to meet the target point
     * @param pose
     * @param side
     * @return
     */
    public static Pose2d poseAdjuster(Pose2d pose, RobotSides side) {
        if (side == RobotSides.CENTER)
            return pose;

        double X = pose.getX(), Y = pose.getY(), H = pose.getHeading(),
            Afb = (robotY / 2) * sin(H), Bfb = (robotY / 2) * cos(H),
            Arl = (robotX / 2) * sin(H), Brl = (robotX / 2) * cos(H);

        switch (side) {
            case FRONT:
                X -= Bfb;
                Y -= Afb;
                break;
            case REAR:
                X += Bfb;
                Y += Afb;
                break;
            case LEFT:
                X += Arl;
                Y -= Brl;
                break;
            default: // RIGHT
                X -= Arl;
                Y += Brl;
                break;
        }

        return new Pose2d(X, Y, H);
    }

    /**
     * Converts the pose of the tip of the extendo (intake) to the local pose of the robot
     * @param pose
     * @param extendoLength
     * @return
     */
    public static Pose2d tipPoseTransfer(Pose2d pose, double extendoLength) {

        double
            x = sin(pose.getHeading()) * (extendoLength + (robotY/2)),
            y = cos(pose.getHeading()) * (extendoLength + (robotY/2));

        return new Pose2d(pose.getX() - x,
                          pose.getY() - y,
                          pose.getHeading());
    }

    public double[] robotCentricMovement(double x, double y, double t) {

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(y) + abs(x) + abs(t), 1);
        double frontLeftPower = (y + x + t) / denominator;
        double backLeftPower = (y - x + t) / denominator;
        double frontRightPower = (y - x - t) / denominator;
        double backRightPower = (y + x - t) / denominator;

        return new double[]{frontLeftPower, backLeftPower, frontRightPower, backRightPower};
    }
}