package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class StackTapeProcessor implements VisionProcessor {
    public Rect rectTape = new Rect();
    public double lightLimitX = 70;  //0-255 for lightness scale in HLS
    public double lightLimitY = 70;  //255 is max lightness
    public int tapeROI_1 = 1;
    public int tapeROI_2x = 10;
    public int tapeROI_2y = 20;
    Mat submat = new Mat();
    Mat hlsMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }
    int yTapeBottomEdge = 0;  //top of image if not found
    int xTapeLeft = 0;
    int xTapeRight = 639;

    public double bottomLightness;
    public double maxLightVal;

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hlsMat, Imgproc.COLOR_RGB2HLS);

        Mat lightMat = new Mat();
        Core.extractChannel(hlsMat,lightMat,1);

        bottomLightness = getMaxLightness(lightMat, new Rect(0, 479, 640, tapeROI_1));
        for (int y = 480-tapeROI_1-1; y > 0; y--) {
            maxLightVal = getMaxLightness(lightMat, new Rect(0, y, 640, tapeROI_1));
            if (maxLightVal > bottomLightness+lightLimitY) {
                yTapeBottomEdge = y;
                break;
            }
        }
        int searchRangeTop = yTapeBottomEdge - tapeROI_2y;

        // Corner case handling: too close to the top of the image
        if (searchRangeTop < 0){
            searchRangeTop = 0;
        }

        // Open up a horizontal window for detection the left and right edge of the tape
        Rect whiteTapeSearchRange = new Rect(0, searchRangeTop, 640, tapeROI_2y);
        Mat subHlsMat = hlsMat.submat(whiteTapeSearchRange);

        for (int x = 0; x < 640-tapeROI_2x-1; x++) {
            double avgLightVal = getAvgLightness(subHlsMat, new Rect(x, 0, tapeROI_2x, tapeROI_2y));
            if (avgLightVal > bottomLightness+lightLimitX) {
                xTapeLeft = x;
                break;
            }
        }

        for (int x = 640-tapeROI_2x-1; x > 0; x--) {
            double avgLightVal = getAvgLightness(subHlsMat, new Rect(x, 0, tapeROI_2x, tapeROI_2y));
            if (avgLightVal > bottomLightness+lightLimitX) {
                xTapeRight = x + tapeROI_2x;
                break;
            }
        }

        int roiWidth = xTapeRight - xTapeLeft + 1;

        if(roiWidth <1){
            roiWidth = 1;
        }

        rectTape = new Rect(xTapeLeft, yTapeBottomEdge, roiWidth, tapeROI_1);
        return rectTape;
    }

    protected double getAvgLightness(Mat input, Rect rect){
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1]; // Lightness
    }

    protected double getMaxLightness(Mat input, Rect rect){
        submat = input.submat(rect);
        Core.MinMaxLocResult minMaxLightVal = Core.minMaxLoc(submat);
        return minMaxLightVal.maxVal; // Lightness
    }

    private android.graphics.Rect makeAndroidRect(Rect rect, float scaleBmpPxToCanvasPx){
        int left = Math.round(rect.x*scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y*scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width*scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height*scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left,top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint rectPaint = new Paint();
        rectPaint.setColor(Color.MAGENTA);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity*12);

        android.graphics.Rect drawRectTape = makeAndroidRect(rectTape, scaleBmpPxToCanvasPx);
        canvas.drawRect(drawRectTape, rectPaint);
    }
}
