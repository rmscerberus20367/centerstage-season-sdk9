package org.firstinspires.ftc.teamcode.auton.auton2;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RobotLocalization;
import org.firstinspires.ftc.teamcode.SpikeProcessor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.function.ArmWrist;
import org.firstinspires.ftc.teamcode.function.Grip;
import org.firstinspires.ftc.teamcode.function.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


public class BlueNear extends LinearOpMode {
    Grip claw = new Grip();
    ArmWrist arm = new ArmWrist();
    Slides slides = new Slides();

    Pose2d startPose = new Pose2d(12, 60, Math.toRadians(90));

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
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)

                .back(36)
                .turn(Math.toRadians(90))

                .build();
        drive.setPoseEstimate(startPose);
        waitForStart();
        if (opModeIsActive()){
            drive.followTrajectorySequence(traj1);

            AprilTagDetection tag = tagProcessor.getDetections().get(1);
            if (tag.ftcPose.x != 0){
                double[] a = RobotLocalization.getRobotXY(tag);
                Pose2d pose2 = new Pose2d(12, 60, Math.toRadians(90));


            }




        }
    }
}
