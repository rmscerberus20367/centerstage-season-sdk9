
package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@TeleOp
public class MyOpMode extends LinearOpMode {
    private int targetTagID = 0;
    @Override
    public void runOpMode() {
        SpikeProcessor spikeProcessor = new SpikeProcessor();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose=new Pose2d(12, -60, Math.toRadians(270));
        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlide = hardwareMap.dcMotor.get("rightSlide");
        Servo gripTilt = hardwareMap.servo.get("intakeTilt");
        Servo depositTiltLeft = hardwareMap.servo.get("depositTiltLeft");
        Servo depositTiltRight = hardwareMap.servo.get("depositTiltRight");
        Servo gripLeft = hardwareMap.servo.get("gripLeft");
        Servo gripRight = hardwareMap.servo.get("gripRight");
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .turn(Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    gripTilt.setPosition(0);
                })
                .lineToLinearHeading(new Pose2d(12, -40, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    gripLeft.setPosition(0.5);
                })
                .waitSeconds(4)
                .lineToLinearHeading(new Pose2d(48, -40, Math.toRadians(180)))
                .waitSeconds(3)
                .forward(3)
                .strafeRight(27)
                .back(7)
                .build();

        gripLeft.setPosition(0.325);
        depositTiltRight.setPosition(0.7);
        depositTiltLeft.setPosition(0.3);
        gripTilt.setPosition(0.5);
        while (!isStarted() && !isStopRequested()){

        }
        waitForStart();

        if(isStopRequested()) return;

        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(traj);
    }
}