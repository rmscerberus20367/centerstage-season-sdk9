package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SpikeProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class Vision extends LinearOpMode {
    private AprilTagDetection targetTag= null;
    private int targetTagID = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SpikeProcessor spikeProcessor = new SpikeProcessor();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(974.173, 974.173,277.586,251.531)
                .build();


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("SpikeAnswer", spikeProcessor.getAnswer());

            switch (spikeProcessor.getAnswer()){
                case LEFT:
                    targetTagID = 1;
                    break;
                case CENTER:
                    targetTagID = 2;
                    break;
                case RIGHT:
                    targetTagID = 3;
                    break;
                case NONE:
                    break;
            }
            telemetry.addData("r", spikeProcessor.satRectRight);
            telemetry.addData("c", spikeProcessor.satRectCenter);
            telemetry.update();
            if (tagProcessor.getDetections().size() > 0){
                for (int i = 0; i<tagProcessor.getDetections().size(); i++) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(i);
                    if (tag.id == targetTagID || tag.id == targetTagID + 3) {
                        telemetry.addData("ID", tag.id);
                        telemetry.addData("x", tag.ftcPose.x);
                        telemetry.addData("y", tag.ftcPose.y);
                        telemetry.addData("z", tag.ftcPose.z);
                        telemetry.addData("roll", tag.ftcPose.roll);
                        telemetry.addData("pitch", tag.ftcPose.pitch);
                        telemetry.addData("yaw", tag.ftcPose.yaw);
                        telemetry.addData("range", tag.ftcPose.range );
                    }
                }
            }


            telemetry.update();
        }
    }
}





