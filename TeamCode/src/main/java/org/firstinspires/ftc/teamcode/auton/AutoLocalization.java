
package org.firstinspires.ftc.teamcode.auton;
import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class AutoLocalization extends LinearOpMode {
    private int targetTagID = 2;
    private double xyScaling = 1.00;
    private double rangeCorrection = 0; //inches

    private double destinationX = 18;
    private double destinationY = 0;
    private double pGain = 0.75;
    private double  dCamera = 5.0; //distance from center of robot to camera

    private int correctionCounter = 0;

//    private double tagX = 24;//62.34;  //inches
//    private double tagY1 = 5;//41.65; //blue left tagID = 1
//    private double tagY2 = 0;//35.60; //blue center tagID = 2
//    private double tagY3 = -5;//29.61; //blue right tagID = 3
//    private double tagY4 = -29.61; //red left tagID = 4
//    private double tagY5 = -35.60; //red center tagID = 5
//    private double tagY6 = -62.34; //red right tagID = 6

//    private double[] getRobotXY(AprilTagDetection tag) {
//        double[] robotXY = new double[8];
//        double tagY = 0; //inches
//        if (tag.id==1){
//            tagY = tagY1;}
//        else if (tag.id==2) {
//            tagY = tagY2;}
//        else if (tag.id==3) {
//            tagY = tagY3;}
//        else if (tag.id==4) {
//            tagY = tagY4;}
//        else if (tag.id==5) {
//            tagY = tagY5;}
//        else if (tag.id==6) {
//            tagY = tagY6;}
//
//        // get range, bearing, and yaw from april tag
//        double rangeAT = tag.ftcPose.range + rangeCorrection;
//        double bearingAT = Math.toRadians(tag.ftcPose.bearing);
//        double yawAT = Math.toRadians(tag.ftcPose.yaw);
//
//        //calculating deltaX and deltaY of the tag relative to the camera
//        double camTagX = rangeAT * Math.cos(bearingAT - yawAT);
//        double camTagY = rangeAT * Math.sin(bearingAT - yawAT);
//
//        //calculating deltaX and deltaY of the camera relative to the center of robot
//        double camX = dCamera * Math.cos(-yawAT);
//        double camY = dCamera * Math.sin(-yawAT);
//
//        //calculating relative X and relative Y of the robot to the tag
//        robotXY[0] = xyScaling*(tagX - camTagX - camX);
//        robotXY[1] = xyScaling*(tagY - camTagY - camY);
//        robotXY[2] = tagX;
//        robotXY[3] = camTagX;
//        robotXY[4] = camX;
//        robotXY[5] = tagY;
//        robotXY[6] = camTagY;
//        robotXY[7] = camY;
//
//        return robotXY;
//    }

    private double[] getTagXY (AprilTagDetection tag) {
        double[] tagXY = new double[2];

        // get range, bearing, and yaw from april tag
        double rangeAT = tag.ftcPose.range + rangeCorrection;
        double bearingAT = Math.toRadians(tag.ftcPose.bearing);
        double yawAT = Math.toRadians(tag.ftcPose.yaw);

        //calculating deltaX and deltaY of the tag relative to the camera
        double camTagX = rangeAT * Math.cos(bearingAT - yawAT);
        double camTagY = rangeAT * Math.sin(bearingAT - yawAT);

        //calculating deltaX and deltaY of the camera relative to the center of robot
        double camX = dCamera * Math.cos(-yawAT);
        double camY = dCamera * Math.sin(-yawAT);

        tagXY[0] = xyScaling*(camTagX + camX);
        tagXY[1] = xyScaling*(camTagY + camY);

        return tagXY;
    }

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, 60, Math.toRadians(90));
        Pose2d currentPose = new Pose2d(12, 60, Math.toRadians(90));
        //        SpikeProcessor spikeProcessor = new SpikeProcessor();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(974.173, 974.173,277.586,251.531)
                .build();


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
//                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        ElapsedTime timer = new ElapsedTime();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection tag : currentDetections) {
                if (tag.metadata != null) {
                    if (tag.id == targetTagID){
                        double[] thisTagXY = getTagXY(tag);
                        double errorX = -(thisTagXY[0] - destinationX);
                        double errorY = -(thisTagXY[1] - destinationY);
                        double errorH = tag.ftcPose.yaw;

                        telemetry.addLine(String.format("\nID%d: X %6.1f (in), Y %6.1f (in)", tag.id, thisTagXY[0], thisTagXY[1]));
                        telemetry.addData("yaw", tag.ftcPose.yaw);
                        telemetry.addData("bearing", tag.ftcPose.bearing);
                        telemetry.addData("range", tag.ftcPose.range );
                        telemetry.addData("errorX", errorX);
                        telemetry.addData("errorY", errorY);
                        telemetry.addData("correctionCounter", correctionCounter);
                        telemetry.update();

                        if (correctionCounter==0) {
                            Trajectory myTrajectory1 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(errorH)))
                                    .lineToLinearHeading(new Pose2d(errorX * pGain, errorY * pGain, Math.toRadians(errorH * pGain)))
                                    .build();
                            drive.followTrajectory(myTrajectory1);
                            correctionCounter = correctionCounter + 1;
                            if(isStopRequested()) return;
                            timer.reset();
                            while (timer.seconds()<10){}
                        }
                        else if (correctionCounter==1) {
                            Trajectory myTrajectory2 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(errorH)))
                                    .lineToLinearHeading(new Pose2d(errorX * pGain, errorY * pGain, Math.toRadians(errorH * pGain)))
                                    .build();
                            drive.followTrajectory(myTrajectory2);
                            correctionCounter = correctionCounter + 1;
                            if(isStopRequested()) return;
                            timer.reset();
                            while (timer.seconds()<10){}
                        }
                        else if (correctionCounter==2) {
                            Trajectory myTrajectory3 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(errorH)))
                                    .lineToLinearHeading(new Pose2d(errorX * pGain, errorY * pGain, Math.toRadians(errorH * pGain)))
                                    .build();
                            drive.followTrajectory(myTrajectory3);
                            correctionCounter = correctionCounter + 1;
                            if(isStopRequested()) return;
                            timer.reset();
                            while (timer.seconds()<10){}
                        }

                    }
                }
                else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", tag.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", tag.center.x, tag.center.y));
                }
            }
        }


//        ElapsedTime timer = new ElapsedTime();
//        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
//                .strafeRight(10)
//                .back(5)
//                .turn(Math.toRadians(180))
//
//                .build();
//        TrajectorySequence traj1center = drive.trajectorySequenceBuilder(traj1.end())
//                .lineToLinearHeading(new Pose2d(12, 31, Math.toRadians(270)))
//                .build();
//        TrajectorySequence traj1left = drive.trajectorySequenceBuilder(traj1.end())
//                .lineToLinearHeading(new Pose2d(23, 37, Math.toRadians(270)))
//                .build();
//
//
//        TrajectorySequence traj1right = drive.trajectorySequenceBuilder(traj1.end())
//                .lineToLinearHeading(new Pose2d(21, 35, Math.toRadians(270)))
//                .turn(Math.toRadians(-90))
//                .forward(8)
//
//                .build();


//        while (!isStarted() && !isStopRequested()){
//            telemetry.addData("SpikeAnswer", spikeProcessor.getAnswer());
//            telemetry.update();
//            switch (spikeProcessor.getAnswer()){
//                case LEFT:
//                    targetTagID = 1;
//                    break;
//                case CENTER:
//                    targetTagID = 2;
//                    break;
//                case RIGHT:
//                    targetTagID = 3;
//                    break;
//                case NONE:
//                    break;
//            }
//        }
//
//
//        if(isStopRequested()) return;
//
//        drive.setPoseEstimate(startPose);
//
//        drive.followTrajectorySequence(traj1);
//        gripTilt.setPosition(0);
//        switch (targetTagID){
//            case 1:
//                drive.followTrajectorySequence(traj1left);
//                currentPose = traj1left.end();
//                break;
//            case 2:
//                drive.followTrajectorySequence(traj1center);
//                currentPose = traj1center.end();
//                break;
//            case 3:
//                drive.followTrajectorySequence(traj1right);
//                currentPose = traj1right.end();
//                break;
//        }
//        timer.reset();
//        gripLeft.setPosition(0.5);
//        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(currentPose)
//                .back(5)
//                .addDisplacementMarker(() -> {
//                    gripTilt.setPosition(0.5);
//                    depositTiltRight.setPosition(0);
//                    depositTiltLeft.setPosition(1);
//                    leftSlide.setTargetPosition(625);
//                    rightSlide.setTargetPosition(625);
//                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    leftSlide.setPower(0.5);
//                    rightSlide.setPower(0.5);
//                })
//                .lineToLinearHeading(new Pose2d(52, 32.5, Math.toRadians(180)))
//                .addDisplacementMarker(() ->{gripLeft.setPosition(0.5);})
//                .build();
//        TrajectorySequence traj2right = drive.trajectorySequenceBuilder(traj2.end())
//                .forward(1.5)
//                .strafeLeft(7)
//                .build();
//        TrajectorySequence traj2left = drive.trajectorySequenceBuilder(traj2.end())
//                .strafeRight(8)
//                .build();
//
//        while (timer.seconds()<2){}
//
//        drive.followTrajectorySequence(traj2);
//        switch (targetTagID){
//            case 1:
//                drive.followTrajectorySequence(traj2left);
//                currentPose = traj2left.end();
//                break;
//            case 2:
//                currentPose = traj2.end();
//                break;
//            case 3:
//                drive.followTrajectorySequence(traj2right);
//                currentPose = traj2right.end();
//                break;
//        }
//        gripRight.setPosition(0.5);
//        timer.reset();
//
//        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(currentPose)
//                .forward(10)
//                .lineTo(new Vector2d(50, 8))
//                .back(5)
//                .build();
//        while (timer.seconds()<2){}
//
//
//        drive.followTrajectorySequence(traj3);
//
//        depositTiltRight.setPosition(0.725);
//        depositTiltLeft.setPosition(0.275);
//        leftSlide.setTargetPosition(0);
//        rightSlide.setTargetPosition(0);
//        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftSlide.setPower(0.5);
//        rightSlide.setPower(0.5);
//        timer.reset();
//        gripLeft.setPosition(0.325);
//        gripRight.setPosition(0.675);
//
//        while (timer.seconds()<4){}

    }
}