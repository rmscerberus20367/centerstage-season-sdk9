package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotLocalization;
import org.firstinspires.ftc.teamcode.SpikeProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.function.Average;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp

public class AprilTagTest extends LinearOpMode {
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        drive.setPoseEstimate(startPose);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("pos", drive.getPoseEstimate());

            double[] a = new double[2];
            a[0] = 33;
            a[1] = 32;
            telemetry.addData("array", Average.arrayavg(a));
            telemetry.update();
            if (gamepad1.a) RobotLocalization.getRobotPose(tagProcessor, drive); drive.update();
        }

    }
}
