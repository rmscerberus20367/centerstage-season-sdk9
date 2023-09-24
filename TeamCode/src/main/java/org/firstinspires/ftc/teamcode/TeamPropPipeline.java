package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class TeamPropPipeline extends OpenCvPipeline
{
    boolean viewportPaused;
    int position = 0;
    OpenCvWebcam webcam;
    boolean alliance = false;

    public void setWebcam(OpenCvWebcam webcam1){
        webcam = webcam1;
    }

    public void setAllianceRed(){
        alliance = true;
    }
    @Override
    public Mat processFrame(Mat input)
    {

        List<Mat> rgb = new ArrayList<>();
        Core.split(input, rgb);

        int left = 0, mid = 0, right = 0;
        int minValue = 128;
        int maxValue = 128;

        for (int i=0; i < rgb.get(0).rows(); i++) {
            for (int j=0; j < (rgb.get(0).cols())/3; j++) {
                if (rgb.get(0).get(i,j)[0] > maxValue && rgb.get(1).get(i,j)[0] < minValue && rgb.get(2).get(i,j)[0] < minValue){
                    left++;
                    input.put(i, j, 0, 255, 0, 0);
                }
            }

        }
        for (int i=0; i < rgb.get(0).rows(); i++) {
            for (int j=(rgb.get(0).cols())/3; j < 2*(rgb.get(0).cols())/3; j++) {
                if (rgb.get(0).get(i,j)[0] > maxValue && rgb.get(1).get(i,j)[0] < minValue && rgb.get(2).get(i,j)[0] < minValue){
                    mid++;
                    input.put(i, j, 0, 0, 255, 0);
                }
            }

        }
        for (int i=0; i < rgb.get(0).rows(); i++) {
            for (int j=2*(rgb.get(0).cols())/3; j < rgb.get(0).cols(); j++) {
                if (rgb.get(0).get(i,j)[0] > maxValue && rgb.get(1).get(i,j)[0] < minValue && rgb.get(2).get(i,j)[0] < minValue){
                    right++;
                    input.put(i, j, 0, 255, 0, 0);
                }
            }

        }
        if (left > mid && left > right)
        {
            position = 1;
        }
        else if (mid > left && mid > right)
        {

            position = 3;
        }
        else if (right > mid && right > left)
        {

            position=2;
        }

        return input;
    }
    public int teamPropPosition()
    {
        return position;
    }
    @Override
    public void onViewportTapped()
    {

        viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }
}
