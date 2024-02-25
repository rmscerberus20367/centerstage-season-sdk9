package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp

public class LineTapeDetection extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LineTapeProcessorHough tapeProcessor = new LineTapeProcessorHough();

        VisionPortal visionPortal2 = new VisionPortal.Builder()
                .addProcessor(tapeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("mat", tapeProcessor.lines.rows());
            telemetry.update();
        }
    }
}
