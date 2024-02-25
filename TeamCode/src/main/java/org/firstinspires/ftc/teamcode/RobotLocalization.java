package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.processor.SpikeProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.function.Average;
import org.firstinspires.ftc.teamcode.function.RobotFunction;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;

@TeleOp
public class RobotLocalization extends LinearOpMode {
    private AprilTagDetection targetTag= null;
    private int targetTagID = 0;

    private static double xyScaling = 1.00;
    private static double rangeCorrection = 0; //inches
    private static double tagX = 62.34;//62.34;  //inches
    private static double tagY1 = 41.65;//41.65; //blue left tagID = 1
    private static double tagY2 = 35.60;//35.60; //blue center tagID = 2
    private static double tagY3 = 29.61;//29.61; //blue right tagID = 3
    private static double tagY4 = -29.61; //red left tagID = 4
    private static double tagY5 = -35.60; //red center tagID = 5
    private static double tagY6 = -41.65; //red right tagID = 6
    private static double  dCamera = 8.5; //distance from center of robot to camera

    public static double[] getRobotXY(AprilTagDetection tag) {
        double[] robotXY = new double[8];
        double tagY = 0; //inches
        if (tag.id==1){
            tagY = tagY1;}
        else if (tag.id==2) {
            tagY = tagY2;}
        else if (tag.id==3) {
            tagY = tagY3;}
        else if (tag.id==4) {
            tagY = tagY4;}
        else if (tag.id==5) {
            tagY = tagY5;}
        else if (tag.id==6) {
            tagY = tagY6;}

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

        //calculating relative X and relative Y of the robot to the tag
        robotXY[0] = xyScaling*(tagX - camTagX - camX);
        robotXY[1] = xyScaling*(tagY - camTagY - camY);
        robotXY[2] = tagX;
        robotXY[3] = camTagX;
        robotXY[4] = camX;
        robotXY[5] = tagY;
        robotXY[6] = camTagY;
        robotXY[7] = camY;

        return robotXY;
    }
    public static Pose2d getRobotPose(AprilTagProcessor tagProcessor, SampleMecanumDrive drive) {
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        int numOfTags = 0;
        for (AprilTagDetection tag : currentDetections) {
            if (tag.metadata != null) {
                numOfTags++;
            }
        }
        if (numOfTags == 0){
            return drive.getPoseEstimate();

        }
        else {
            double[] robotX = new double[numOfTags];
            double[] robotY = new double[numOfTags];

            int curTag = 0;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.metadata != null) {
                    double[] robotXYthisTag = getRobotXY(tag);


                    robotX[curTag] = robotXYthisTag[0];
                    robotY[curTag] = robotXYthisTag[1];
                    curTag++;


                }
            }
            double x = Average.arrayavg(robotX);
            double y = Average.arrayavg(robotY);

            double h = drive.getPoseEstimate().getHeading();
            Pose2d atagPose = new Pose2d(x, y, h);
            drive.setPoseEstimate(atagPose);

            return atagPose;
        }
    }
    public static Pose2d getRobotPose(AprilTagProcessor tagProcessor, SampleMecanumDriveCancelable drive) {
        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();

        int numOfTags = 0;
        for (AprilTagDetection tag : currentDetections) {
            if (tag.metadata != null) {
                numOfTags++;
            }
        }
        if (numOfTags == 0){
            return drive.getPoseEstimate();

        }
        else {
            double[] robotX = new double[numOfTags];
            double[] robotY = new double[numOfTags];

            int curTag = 0;
            for (AprilTagDetection tag : currentDetections) {
                if (tag.metadata != null) {
                    double[] robotXYthisTag = getRobotXY(tag);


                    robotX[curTag] = robotXYthisTag[0];
                    robotY[curTag] = robotXYthisTag[1];
                    curTag++;


                }
            }
            double x = Average.arrayavg(robotX);
            double y = Average.arrayavg(robotY);

            double h = drive.getPoseEstimate().getHeading();
            Pose2d atagPose = new Pose2d(x, y, h);
            drive.setPoseEstimate(atagPose);

            return atagPose;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        SpikeProcessor spikeProcessor = new SpikeProcessor();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(974.173, 974.173,277.586,251.531)
                .build();
        RobotFunction robot  = new RobotFunction();
        robot.init(hardwareMap);


        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
//                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection tag : currentDetections) {
                if (tag.metadata != null) {
                    double[] robotXYthisTag = getRobotXY(tag);
                    telemetry.addLine(String.format("\nID%d: X %6.1f (in), Y %6.1f (in)", tag.id, robotXYthisTag[0], robotXYthisTag[1]));
                    telemetry.addData("tagX", robotXYthisTag[2]);
                    telemetry.addData("camTagX", robotXYthisTag[3]);
                    telemetry.addData("camX", robotXYthisTag[4]);
                    telemetry.addData("tagY", robotXYthisTag[5]);
                    telemetry.addData("camTagY", robotXYthisTag[6]);
                    telemetry.addData("camY", robotXYthisTag[7]);



                    telemetry.addData("yaw", tag.ftcPose.yaw);
                    telemetry.addData("bearing", tag.ftcPose.bearing);
                    telemetry.addData("range", tag.ftcPose.range );
                }
                else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", tag.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", tag.center.x, tag.center.y));
                }
            }
            robot.update(gamepad1, gamepad1);

            telemetry.update();
        }
    }
}