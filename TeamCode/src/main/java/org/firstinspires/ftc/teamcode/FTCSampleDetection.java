package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;

import org.opencv.core.Core;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class FTCSampleDetection implements VisionProcessor {
    Rect LEFT_RECTANCLE;
    Rect MIDDLE_RECTANGLE;
    Rect RIGHT_RECTANGLE;

    Mat HSV_MAT = new Mat();
    Mat lowMat = new Mat();
    Mat highMat = new Mat();
    Mat detectedMat = new Mat();
    Scalar lowerRedLow = new Scalar(0,125, 125);
    Scalar lowerRedHigh = new Scalar(10,255,255);
    //Opencv does 0 to 180, not 0 to 360. Something to look out online
    Scalar higherRedLow = new Scalar(165,125, 125);
    Scalar higherRedHigh = new Scalar(180,255,255);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        LEFT_RECTANCLE = new Rect(
                new Point(0,0),
                new Point(0.33* width,height)
        );
        MIDDLE_RECTANGLE = new Rect(
                new Point(0.33 * width, 0),
                new Point(0.66 * width, height)
        );
        RIGHT_RECTANGLE = new Rect(
                new Point(0.66 * width, 0),
                new Point(width, height)

        );
        //Init all the rectangles to split up the frame
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame,HSV_MAT, Imgproc.COLOR_RGB2HSV);
        //Converts RGB frame to HSV
        Core.inRange(HSV_MAT, lowerRedLow, lowerRedHigh, lowMat);
        Core.inRange(HSV_MAT, higherRedLow, higherRedHigh,highMat);

        Core.bitwise_or(lowMat, highMat,detectedMat);
        double leftPercent = (Core.sumElems(detectedMat.submat(LEFT_RECTANCLE)).val[0] / 255);
        double middlePercent = (Core.sumElems(detectedMat.submat(MIDDLE_RECTANGLE)).val[0] / 255);
        double rightPercent = (Core.sumElems(detectedMat.submat(RIGHT_RECTANGLE)).val[0] / 255);
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
