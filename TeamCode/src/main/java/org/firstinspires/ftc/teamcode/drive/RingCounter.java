package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


@Config
public class RingCounter extends OpenCvPipeline {

    public static double lowH = 5;
    public static double lowS = 120;
    public static double lowV = 100;

    public static double highH = 10;
    public static double highS = 255;
    public static double highV = 255;

    Scalar lowHSV = new Scalar(lowH, lowS, lowV);
    Scalar highHSV = new Scalar(highH, highS, highV);

    public static double ringThres0 = 5;
    public static double ringThres1 = 25;

    public int count;

    public double meanVal;

    Mat cameraInput;
    Mat thresh;
    Mat ringBoxPixels;

    public RingCounter() {
        cameraInput = new Mat();
        thresh = new Mat();
    }

    @Override
    public Mat processFrame(Mat input) {
        // HSV version of camera input
        Imgproc.cvtColor(input, cameraInput, Imgproc.COLOR_RGB2HSV);
        Core.inRange(cameraInput, lowHSV, highHSV, thresh);

        Rect ringBox = new Rect(40, 50, 80, 60);

        //Imgproc.rectangle(thresh, ringBox, new Scalar(78, 255, 130));

        ringBoxPixels = new Mat(thresh, ringBox);
        meanVal = Core.mean(ringBoxPixels).val[0];

        if (meanVal < ringThres0) {
            count = 0;
        }
        else if (meanVal < ringThres1) {
            count = 1;
        }
        else {
            count = 4;
        }

        return ringBoxPixels;
    }
    public int getRingCount() {
        return this.count;
    }
}
