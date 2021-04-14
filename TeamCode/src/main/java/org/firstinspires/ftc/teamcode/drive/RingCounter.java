package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


@Config
public class RingCounter extends OpenCvPipeline {

    private FtcDashboard dashboard;
    public static double lowH = 5;
    public static double lowS = 120;
    public static double lowV = 150;

    public static double highH = 10;
    public static double highS = 255;
    public static double highV = 255;

    public static double lineThreshLow = 100;
    public static double lineThreshHigh = 300;

    public int count;
    // Image width/height
    public int width;
    public int height;

    public RingCounter(int width, int height) {
        this.width = width;
        this.height = height;
    }

    @Override
    public Mat processFrame(Mat input) {

        // HSV version of camera input
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(lowH, lowS, lowV);
        Scalar highHSV = new Scalar(highH, highS, highV);

        Mat thresh = new Mat();
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Mat edges = new Mat();
        // Imgproc.Canny(thresh, edges, lineThreshLow, lineThreshHigh);

        Rect ringBox = new Rect(60, 70, 55, 40);

        Imgproc.rectangle(thresh, ringBox, new Scalar(78, 255, 130));

        return thresh;

    }
}
