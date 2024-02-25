package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.vision.VisionPortal.MultiPortalLayout.HORIZONTAL;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.StackTapeProcessor;
import org.firstinspires.ftc.teamcode.SpikeProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class StackDetection extends LinearOpMode {
    private AprilTagDetection targetTag= null;
    private int targetTagID = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init multi-portal
        int[] myPortalList = VisionPortal.makeMultiPortalView(2, HORIZONTAL);
        int portal1ViewID = myPortalList[0];
        int portal2ViewID = myPortalList[1];


        // Init Spike and Stack Processor
        StackTapeProcessor tapeProcessor = new StackTapeProcessor();
        SpikeProcessor spikeProcessor = new SpikeProcessor();

        // Init AprilTag Processors
        AprilTagProcessor tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(974.173, 974.173,277.586,251.531)
                .build();

        AprilTagProcessor tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(825.850,825.850,326.316,244.585)
                .build();

        // Init Multi VisionPortal
        VisionPortal visionPortal1 = new VisionPortal.Builder()
                .addProcessor(tagProcessor1)
                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(portal1ViewID)
                .build();

        VisionPortal visionPortal2 = new VisionPortal.Builder()
                .addProcessor(tagProcessor2)
                .addProcessor(tapeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(portal2ViewID)
                .build();


        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("yTapeBottomEdge", tapeProcessor.yTapeBottomEdge);
            telemetry.addData("xTapeLeft", tapeProcessor.xTapeLeft);
            telemetry.addData("xTapeRight", tapeProcessor.xTapeRight);


            telemetry.addData("SpikeAnswer", spikeProcessor.getAnswer());

            telemetry.addData("bottomlightval", tapeProcessor.bottomLightness);
            telemetry.addData("maxLightVal", tapeProcessor.maxLightVal);
            telemetry.addData("difference", tapeProcessor.maxLightVal-tapeProcessor.bottomLightness);

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

//            if (tagProcessor1.getDetections().size() > 0){
//                for (int i = 0; i<tagProcessor1.getDetections().size(); i++) {
//                    AprilTagDetection tag = tagProcessor1.getDetections().get(i);
//                    if (tag.id == targetTagID || tag.id == targetTagID + 3) {
//                        telemetry.addData("ID", tag.id);
//                        telemetry.addData("x", tag.ftcPose.x);
//                        telemetry.addData("y", tag.ftcPose.y);
//                        telemetry.addData("z", tag.ftcPose.z);
//                        telemetry.addData("roll", tag.ftcPose.roll);
//                        telemetry.addData("pitch", tag.ftcPose.pitch);
//                        telemetry.addData("yaw", tag.ftcPose.yaw);
//                        telemetry.addData("range", tag.ftcPose.range );
//                    }
//                }
//            }

            if (tagProcessor1.getDetections().size() > 0){
                AprilTagDetection tag1 = tagProcessor1.getDetections().get(0);
                telemetry.addData("ID1", tag1.id);
                telemetry.addData("x1", tag1.ftcPose.x);
                telemetry.addData("y1", tag1.ftcPose.y);
                telemetry.addData("z1", tag1.ftcPose.z);
                telemetry.addData("roll1", tag1.ftcPose.roll);
                telemetry.addData("pitch1", tag1.ftcPose.pitch);
                telemetry.addData("yaw1", tag1.ftcPose.yaw);
            }

            if (tagProcessor2.getDetections().size() > 0){
                AprilTagDetection tag2 = tagProcessor2.getDetections().get(0);
                telemetry.addData("ID2", tag2.id);
                telemetry.addData("x2", tag2.ftcPose.x);
                telemetry.addData("y2", tag2.ftcPose.y);
                telemetry.addData("z2", tag2.ftcPose.z);
                telemetry.addData("roll2", tag2.ftcPose.roll);
                telemetry.addData("pitch2", tag2.ftcPose.pitch);
                telemetry.addData("yaw2", tag2.ftcPose.yaw);
            }

            telemetry.update();
        }
    }
}

