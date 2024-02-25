package org.firstinspires.ftc.teamcode.auton.auton4;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SpikeProcessor2;
import org.firstinspires.ftc.teamcode.StackTapeProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class BlueFar0 extends LinearOpMode {
    far0 auton = new far0();
    @Override
    public void runOpMode() throws InterruptedException {
        SpikeProcessor2 spikeProcessor = new SpikeProcessor2();
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(974.173, 974.173,277.586,251.531)
                .build();
        StackTapeProcessor tapeProcessor = new StackTapeProcessor();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        List myPortalsList;
        int Portal_1_View_ID;
        int Portal_2_View_ID;
        myPortalsList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        Portal_1_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 0, false)).intValue();
        Portal_2_View_ID = ((Integer) JavaUtil.inListGet(myPortalsList, JavaUtil.AtMode.FROM_START, 1, false)).intValue();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))

                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(Portal_1_View_ID)
                .build();
        VisionPortal visionPortal1 = new VisionPortal.Builder()
                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))

                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(Portal_2_View_ID)
                .build();




        auton.init(drive, hardwareMap, tagProcessor);
        auton.create(far0.Alliance.BLUE);


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


        }
    }
}
