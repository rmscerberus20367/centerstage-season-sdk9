package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SpikeProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="BlueFarPath")

public class BlueFarPath extends LinearOpMode {
    private int targetTagID = 2;
    @Override
    public void runOpMode() {
        SpikeProcessor spikeProcessor = new SpikeProcessor();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(90));
        Pose2d currentPose = new Pose2d(-36, 60, Math.toRadians(90));

        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlide = hardwareMap.dcMotor.get("rightSlide");
        Servo gripTilt = hardwareMap.servo.get("intakeTilt");
        Servo depositTiltLeft = hardwareMap.servo.get("depositTiltLeft");
        Servo depositTiltRight = hardwareMap.servo.get("depositTiltRight");
        Servo gripLeft = hardwareMap.servo.get("gripLeft");
        Servo gripRight = hardwareMap.servo.get("gripRight");
        VisionPortal visionPortal = new VisionPortal.Builder()
                //.addProcessor(tagProcessor)
                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        ElapsedTime timer = new ElapsedTime();
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .turn(Math.toRadians(180))
                .addTemporalMarker(2, () -> {
                    gripTilt.setPosition(0);
                })
                .build();
        TrajectorySequence traj1center = drive.trajectorySequenceBuilder(traj.end())
                .forward(12)
                .build();
        TrajectorySequence traj1right = drive.trajectorySequenceBuilder(traj.end())
                .lineTo(new Vector2d(-48, -48))
                .build();
        TrajectorySequence traj1left = drive.trajectorySequenceBuilder(traj.end())
                .forward(18)
                .turn(Math.toRadians(90))
                .forward(4)
                .addDisplacementMarker(()->{
                    gripRight.setPosition(0.5);
                })
                .back(12)
                .build();



        gripLeft.setPosition(0.325);
        gripRight.setPosition(0.675);
        depositTiltRight.setPosition(0.7);
        depositTiltLeft.setPosition(0.3);
        gripTilt.setPosition(0.6);
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("SpikeAnswer", spikeProcessor.getAnswer());
            telemetry.update();
            switch (spikeProcessor.getAnswer()) {
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


        if (isStopRequested()) return;

        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(traj);
        gripTilt.setPosition(0);
        switch (targetTagID){
            case 1: drive.followTrajectorySequence(traj1left);
                currentPose = traj1left.end();
                break;
            case 2:
                drive.followTrajectorySequence(traj1center);
                currentPose = traj1center.end();
                break;
            case 3:
                drive.followTrajectorySequence(traj1right);
                currentPose = traj1right.end();
                break;
        }
        timer.reset();
        gripRight.setPosition(0.5);
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(currentPose)
                .lineToLinearHeading(new Pose2d(-36, 60, Math.toRadians(180)))
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .back(55)
                .addDisplacementMarker(() -> {
                    gripTilt.setPosition(0.5);
                    depositTiltRight.setPosition(0);
                    depositTiltLeft.setPosition(1);
                    leftSlide.setTargetPosition(625);
                    rightSlide.setTargetPosition(625);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlide.setPower(0.5);
                    rightSlide.setPower(0.5);
                })
                .lineToLinearHeading(new Pose2d(58.5, 39, Math.toRadians(180)))
                .build();
        TrajectorySequence traj3right = drive.trajectorySequenceBuilder(traj3.end())
                .strafeLeft(7)
                .build();
        TrajectorySequence traj3left = drive.trajectorySequenceBuilder(traj3.end())
                .strafeRight(8)
                .build();
        while (timer.seconds()<2){}
        drive.followTrajectorySequence(traj2);
        switch (targetTagID){
            case 1:
                drive.followTrajectorySequence(traj3left);
                currentPose = traj3left.end();
                break;
            case 2:
                currentPose = traj3.end();
                break;
            case 3:
                drive.followTrajectorySequence(traj3right);
                currentPose = traj3right.end();
                break;
        }

    }
}
