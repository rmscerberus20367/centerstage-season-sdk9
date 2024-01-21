package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SpikeProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Disabled


public class RedFarPath extends LinearOpMode {
    private int targetTagID = 2;
    int wait_time = 5;
    @Override
    public void runOpMode() {
        SpikeProcessor spikeProcessor = new SpikeProcessor();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(270));
        Pose2d currentPose = new Pose2d(-36, -60, Math.toRadians(270));

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
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(10)
                .back(5)
                .turn(Math.toRadians(180))

                .build();
        TrajectorySequence traj1center = drive.trajectorySequenceBuilder(traj1.end())
                .forward(23.5)
                .strafeRight(10)
                .build();
        TrajectorySequence traj1left = drive.trajectorySequenceBuilder(traj1.end())
                .strafeLeft(1)
                .forward(20)
                .build();
        TrajectorySequence traj1right = drive.trajectorySequenceBuilder(traj1.end())
                .forward(21)
                .turn(Math.toRadians(-90))
                .forward(10)
                .build();


        gripLeft.setPosition(0.325);
        gripRight.setPosition(0.675);
        depositTiltRight.setPosition(0.725);
        depositTiltLeft.setPosition(0.275);
        gripTilt.setPosition(0.6);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        timer.reset();
        while (timer.seconds()<wait_time){}
        drive.followTrajectorySequence(traj1);
        gripTilt.setPosition(0);
        switch (targetTagID) {
            case 1:
                drive.followTrajectorySequence(traj1left);
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
        gripLeft.setPosition(0.5);
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(currentPose)
                .back(12)
                .lineToLinearHeading(new Pose2d(-38, -55.25, Math.toRadians(180)))
                .back(50)
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
                .lineToLinearHeading(new Pose2d(50, -31.5
                        , Math.toRadians(180)))
                .build();
        TrajectorySequence traj2right = drive.trajectorySequenceBuilder(traj2.end())
                .forward(1.5)
                .strafeLeft(7.5)

                .build();
        TrajectorySequence traj2left = drive.trajectorySequenceBuilder(traj2.end())
                .strafeRight(9)
                .build();

        while (timer.seconds()<2){}

        drive.followTrajectorySequence(traj2);
        switch (targetTagID){
            case 1:
                drive.followTrajectorySequence(traj2left);
                currentPose = traj2left.end();
                break;
            case 2:
                currentPose = traj2.end();
                break;
            case 3:
                drive.followTrajectorySequence(traj2right);
                currentPose = traj2right.end();
                break;
        }
        gripRight.setPosition(0.5);
        timer.reset();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(currentPose)
                .forward(5)
                .build();
        while (timer.seconds()<2){}


        drive.followTrajectorySequence(traj3);

        depositTiltRight.setPosition(0.725);
        depositTiltLeft.setPosition(0.275);
        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.5);
        rightSlide.setPower(0.5);
        timer.reset();
        gripLeft.setPosition(0.325);
        gripRight.setPosition(0.675);

        while (timer.seconds()<4){}

    }
}

