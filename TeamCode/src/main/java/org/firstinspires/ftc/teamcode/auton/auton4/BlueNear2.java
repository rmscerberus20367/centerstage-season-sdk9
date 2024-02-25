package org.firstinspires.ftc.teamcode.auton.auton4;

import static org.firstinspires.ftc.vision.VisionPortal.MultiPortalLayout.HORIZONTAL;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SpikeProcessor;
import org.firstinspires.ftc.teamcode.StackTapeProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Disabled

@Autonomous
public class BlueNear2 extends LinearOpMode {
    near2 auton = new near2();
    @Override
    public void runOpMode() throws InterruptedException {
        SpikeProcessor spikeProcessor = new SpikeProcessor();
        int[] myPortalList = VisionPortal.makeMultiPortalView(2, HORIZONTAL);
        int portal1ViewID = myPortalList[0];
        int portal2ViewID = myPortalList[1];
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(974.173, 974.173,277.586,251.531)
                .build();
        StackTapeProcessor tapeProcessor = new StackTapeProcessor();
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(portal1ViewID)
                .build();
        AprilTagProcessor tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(825.850,825.850,326.316,244.585)
                .build();
        VisionPortal visionPortal2 = new VisionPortal.Builder()
                .addProcessor(tapeProcessor)
                .addProcessor(tagProcessor2)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(portal2ViewID)
                .build();

        auton.init(hardwareMap, tagProcessor, tapeProcessor);
        auton.create(near0.Alliance.BLUE);


        int targetTagID = 1;
        while (!isStarted() && !isStopRequested()){
            telemetry.addData("SpikeAnswer", spikeProcessor.getAnswer());
            telemetry.update();
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
        }
        if (opModeIsActive()){
            auton.run(targetTagID);
            visionPortal.close();
            visionPortal2.close();

        }
    }
}
