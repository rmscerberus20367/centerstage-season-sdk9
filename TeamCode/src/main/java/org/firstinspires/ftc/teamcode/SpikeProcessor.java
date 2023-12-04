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

public class SpikeProcessor implements VisionProcessor {
    public Rect rectLeft = new Rect( 0,150,40,200);
    public Rect rectCenter = new Rect(229,150,40,200);
    public Rect rectRight = new Rect(599,150,40,200);
    Selected selection = Selected.NONE;
    Mat submat = new Mat();
    Mat hsvMat = new Mat();
    public int answer=0;
    private Scalar avgLeft;
    private Scalar avgCenter;
    private Scalar avgRight;

    public Selected getAnswer() {
        return selection;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }
    Mat chR = new Mat();
    Mat chG = new Mat();
    Mat chB = new Mat();
    double satRectRight;
    double satRectCenter;
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        satRectCenter = getAvgSaturation(hsvMat, rectCenter);
        satRectRight = getAvgSaturation(hsvMat, rectRight);

        if ((satRectRight > 80) && (satRectCenter < 80)) {
            return Selected.RIGHT;
        } else if ((satRectCenter > 50)) {
            return Selected.CENTER;
        }
        return Selected.LEFT;

//        Core.extractChannel(frame, chR, 0);
//        Core.extractChannel(frame, chG, 1);
//        Core.extractChannel(frame, chB, 2);
//        Imgcodecs.imwrite("/sdcard/FIRST/sigle_colorR.png", chR);
//        mgcodecs.imwrite("/sdcard/FIRST/sigle_color1.png", chG);
//        Imgcodecs.imwrite("/sdcard/FIRST/sigle_color2.png", chB);
//        avgLeft = Core.mean(chR.col(0));
//        avgCenter = Core.mean(chR.col(320));
//        avgRight = Core.mean(chR.col(639));
    }
    protected double getAvgSaturation(Mat input, Rect rect){
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1]; // saturation
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
        rectPaint.setColor(Color.GREEN);
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(scaleCanvasDensity*4);

        android.graphics.Rect drawRectLeft = makeAndroidRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectCenter = makeAndroidRect(rectCenter, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectRight = makeAndroidRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext;
        switch (selection){
            case LEFT:
                canvas.drawRect(drawRectLeft, rectPaint);
                break;
            case CENTER:
                canvas.drawRect(drawRectCenter, rectPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectRight, rectPaint);
                break;
            case NONE:
                break;
        }
    }
    public enum Selected {
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }

}





