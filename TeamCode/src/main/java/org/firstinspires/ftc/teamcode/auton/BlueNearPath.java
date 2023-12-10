
package org.firstinspires.ftc.teamcode.auton;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SpikeProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class BlueNearPath extends LinearOpMode {
    private int targetTagID = 2;
    @Override
    public void runOpMode() {
        SpikeProcessor spikeProcessor = new SpikeProcessor();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(90));
        Pose2d currentPose = new Pose2d(12, 60, Math.toRadians(90));

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
                .lineToLinearHeading(new Pose2d(12, 31, Math.toRadians(270)))
                .build();
        TrajectorySequence traj1left = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(25, 37, Math.toRadians(270)))
                .build();
        TrajectorySequence traj1right = drive.trajectorySequenceBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(21, 35, Math.toRadians(270)))
                .turn(Math.toRadians(-90))
                .forward(8)

                .build();


        gripLeft.setPosition(0.325);
        gripRight.setPosition(0.675);
        depositTiltRight.setPosition(0.7);
        depositTiltLeft.setPosition(0.3);
        gripTilt.setPosition(0.7);
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


        if(isStopRequested()) return;

        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(traj1);
        gripTilt.setPosition(0);
        switch (targetTagID){
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
                .back(5)
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
                .lineToLinearHeading(new Pose2d(51, 33.5, Math.toRadians(180)))
                .addDisplacementMarker(() ->{gripLeft.setPosition(0.5);})
                .build();
        TrajectorySequence traj2right = drive.trajectorySequenceBuilder(traj2.end())
                .forward(1)
                .strafeLeft(7)
                .build();
        TrajectorySequence traj2left = drive.trajectorySequenceBuilder(traj2.end())
                .strafeRight(8)
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
                .forward(10)
                .lineTo(new Vector2d(50, 10))
                .back(7)
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

        while (timer.seconds()<4){}

    }
}