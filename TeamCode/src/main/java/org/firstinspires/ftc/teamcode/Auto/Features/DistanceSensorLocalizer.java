package org.firstinspires.ftc.teamcode.Auto.Features;

import static java.lang.Math.acos;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.tan;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class DistanceSensorLocalizer {

    private static final int FIELD_SIZE = 144; // Inches

    private static final double
        SIDE_DIST_OFFSET_X = 0, // Inches
        SIDE_DIST_OFFSET_Y = 0, // Inches
        REAR_DIST_OFFSET_Y = 0; // Inches

    private static double
        x_direction_mltplr = 1,
        y_direction_mltplr = 1;

    public static Vector2d calculateReal2dLocation(Pose2d curr_pose,
                                                        double rear_Dist,
                                                        double side_Dist) {

        if (curr_pose.getX() < 0) x_direction_mltplr = -1;
        if (curr_pose.getY() < 0) y_direction_mltplr = -1;

        double heading = 90 - curr_pose.getHeading();
        double x1 = SIDE_DIST_OFFSET_X / cos(heading);
        double opposite_offset = tan(heading) * SIDE_DIST_OFFSET_X;
        double hypot = hypot(side_Dist, (opposite_offset + SIDE_DIST_OFFSET_Y));
        double point_angle = 180 - acos(hypot/(opposite_offset + SIDE_DIST_OFFSET_Y)) - (90 - heading);
        double x2 = cos(point_angle) * hypot;
        double real_X = (FIELD_SIZE / 2) - x1 + x2;

        double real_Y = (FIELD_SIZE / 2) - cos(heading) * (rear_Dist + REAR_DIST_OFFSET_Y);

        return new Vector2d(x_direction_mltplr * real_X, y_direction_mltplr * real_Y);
    }

    public static Vector2d calculateRealXLocation(Pose2d curr_pose,
                                                        double dist) {

        if (curr_pose.getX() < 0) x_direction_mltplr = -1;

        double heading = 90 - curr_pose.getHeading();
        double x1 = SIDE_DIST_OFFSET_X / cos(heading);
        double opposite_offset = tan(heading) * SIDE_DIST_OFFSET_X;
        double hypot = hypot(dist, (opposite_offset + SIDE_DIST_OFFSET_Y));
        double point_angle = 180 - acos(hypot/(opposite_offset + SIDE_DIST_OFFSET_Y)) - (90 - heading);
        double x2 = cos(point_angle) * hypot;
        double real_X = (FIELD_SIZE / 2) - x1 + x2;

        return new Vector2d(x_direction_mltplr * real_X, curr_pose.getY());
    }

    public static Vector2d calculateRealYLocation(Pose2d curr_pose,
                                                        double dist) {

        if (curr_pose.getY() < 0) y_direction_mltplr = -1;

        double heading = 90 - curr_pose.getHeading();
        double real_Y = (FIELD_SIZE / 2) - cos(heading) * (dist + REAR_DIST_OFFSET_Y);

        return new Vector2d(curr_pose.getX(), y_direction_mltplr * real_Y);
    }
}