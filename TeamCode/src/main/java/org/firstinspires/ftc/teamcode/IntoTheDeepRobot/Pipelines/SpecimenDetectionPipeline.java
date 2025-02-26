package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SpecimenDetectionPipeline extends OpenCvPipeline {
    public enum SpecimenColor {
        RED,
        BLUE
    }
    private SpecimenColor specimenColor;
    int redThreshold = 75, blueThreshold = 70;

    Rect roi = new Rect(0, 20, 320, 140);
    Mat cropped = new Mat();
    Mat hierarchy = new Mat();

    Mat blurred = new Mat();
    Mat redChannel = new Mat();
    Mat greenChannel = new Mat();
    Mat blueChannel = new Mat();

    // ------------------------------------ For Red Detection
    // ----------------------------------- //
    Mat redGreenDif = new Mat();
    Mat redBlueDif = new Mat();
    Mat redGreenThresh = new Mat();
    Mat redBlueThresh = new Mat();

    // ----------------------------------- For Blue Detection
    // ----------------------------------- //
    Mat blueGreenDif = new Mat();
    Mat blueRedDif = new Mat();
    Mat blueGreenThresh = new Mat();
    Mat blueRedThresh = new Mat();

    Mat thresh = new Mat();
    Mat countourMat = new Mat();
    MatOfPoint orderContour = new MatOfPoint();

    double blurAmount = 8.0;

    // ---------------------------------------- Contours
    // ---------------------------------------- //
    List<MatOfPoint> contoursList = new ArrayList<>();
    List<MatOfPoint> filteredContours = new ArrayList<>();

    private double order_specimen_x = 0.0;

    private Telemetry telemetry;

    public SpecimenDetectionPipeline(Telemetry telemetry, SpecimenColor specimenColor) {
        this.telemetry = telemetry;
        this.specimenColor = specimenColor;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Reset
        contoursList.clear();
        filteredContours.clear();
        orderContour = null;

        // Crop
//        input.submat(roi).copyTo(cropped);
        cropped = input.clone();
        Imgproc.blur(cropped, blurred, new Size(blurAmount, blurAmount));

        // Split Channels
        Core.extractChannel(blurred, redChannel, 0);
        Core.extractChannel(blurred, greenChannel, 1);
        Core.extractChannel(blurred, blueChannel, 2);

        // Threshold
        if (specimenColor == SpecimenColor.RED) {
            Core.subtract(redChannel, greenChannel, redGreenDif);
            Core.subtract(redChannel, blueChannel, redBlueDif);

            Core.compare(redGreenDif, new Scalar(redThreshold, redThreshold),
                    redGreenThresh, Core.CMP_GT);
            Core.compare(redBlueDif, new Scalar(redThreshold, redThreshold),
                    redBlueThresh, Core.CMP_GT);

            Core.bitwise_and(redGreenThresh, redBlueThresh, thresh);
        } else {
            Core.subtract(blueChannel, greenChannel, blueGreenDif);
            Core.subtract(blueChannel, redChannel, blueRedDif);

            Core.compare(blueGreenDif, new Scalar(blueThreshold, blueThreshold),
                    blueGreenThresh, Core.CMP_GT);
            Core.compare(blueRedDif, new Scalar(blueThreshold, blueThreshold),
                    blueRedThresh, Core.CMP_GT);

            Core.bitwise_and(blueGreenThresh, blueRedThresh, thresh);
        }

        // Find Contours
        Imgproc.findContours(thresh, contoursList, hierarchy, Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contoursList) {
            if (Imgproc.contourArea(contour) > 1) {
                filteredContours.add(contour);
            }
        }

        countourMat = cropped.clone();
        Imgproc.drawContours(countourMat, filteredContours, -1, new Scalar(0, 255, 0), 2);
        // Bounding Rectangles
        for (MatOfPoint contour : filteredContours) {
            Rect rect = Imgproc.boundingRect(contour);
            if (contourCenter(contour).x > contourCenter(orderContour).x) {
                orderContour = contour;
            }
            Imgproc.rectangle(countourMat, rect, new Scalar(0, 0, 255), 2);
        }

        if (orderContour != null) {
            Imgproc.rectangle(
                    countourMat,
                    Imgproc.boundingRect(orderContour),
                    new Scalar(255, 255, 0),
                    2
            );
            order_specimen_x = contourCenter(orderContour).x;
        }

        telemetry.addData("Contours Found", filteredContours.size());
        telemetry.addData("Priority Specimen X", getPrioritySpecimenX());

        // return new Mat(input, roi);
        return countourMat;
    }

    public Point contourCenter(MatOfPoint contour) {
        if (contour == null) return new Point(0, 0);
        Rect rect = Imgproc.boundingRect(contour);
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }

    public double getPrioritySpecimenX() {
        return order_specimen_x;
    }
}
