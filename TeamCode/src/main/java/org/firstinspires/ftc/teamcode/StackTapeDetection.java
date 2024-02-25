package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.vision.VisionPortal.MultiPortalLayout.HORIZONTAL;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class StackTapeDetection extends LinearOpMode {


    public static void tapeAlign(StackTapeProcessor tapeProcessor, SampleMecanumDriveCancelable drive){
        int curPos =0;
        int difference;
        Trajectory trajleft = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(12, SampleMecanumDriveCancelable.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory trajright = drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(12, SampleMecanumDriveCancelable.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        curPos = (tapeProcessor.xTapeRight+tapeProcessor.xTapeLeft)/2;


        difference = curPos - 320;


        if (difference > 0) {
            drive.followTrajectoryAsync(trajright);

        } else if (difference < 0) {
            drive.followTrajectoryAsync(trajleft);
        }

        curPos = (tapeProcessor.xTapeRight+tapeProcessor.xTapeLeft)/2;
        difference = curPos - 320;
        while ((Math.abs(difference)>10)&&drive.isBusy()){
            curPos = (tapeProcessor.xTapeRight+tapeProcessor.xTapeLeft)/2;
            difference = curPos - 320;
            drive.update();
        }
        if (Math.abs(difference)<=10){

            drive.breakFollowing();


            // Stop the motors
            drive.setDrivePower(new Pose2d());
            drive.update();
        }

        curPos = 0;
        difference = 0;
        Trajectory trajectory = null;
        curPos = tapeProcessor.yTapeBottomEdge;
        difference = curPos - 354;
        Trajectory trajback = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(12, SampleMecanumDriveCancelable.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        Trajectory trajforward = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(12, SampleMecanumDriveCancelable.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();



        curPos = tapeProcessor.yTapeBottomEdge;


        difference = curPos - 354;


        if (difference > 0) {
            drive.followTrajectoryAsync(trajback);

        } else if (difference < 0) {
            drive.followTrajectoryAsync(trajforward);
        }

        curPos = tapeProcessor.yTapeBottomEdge;
        difference = curPos - 354;
        while ((Math.abs(difference)>10)&&drive.isBusy()){
            curPos = tapeProcessor.yTapeBottomEdge;
            difference = curPos - 354;

            drive.update();
        }
        if (Math.abs(difference)<=10){

            drive.breakFollowing();


            // Stop the motors
            drive.setDrivePower(new Pose2d());
            drive.update();
        }

    }

    @Override

    public void runOpMode() throws InterruptedException {
        int[] myPortalList = VisionPortal.makeMultiPortalView(2, HORIZONTAL);
        int portal1ViewID = myPortalList[0];
        int portal2ViewID = myPortalList[1];


        // Init Spike and Stack Processor
        StackTapeProcessor tapeProcessor = new StackTapeProcessor();
        SpikeProcessor spikeProcessor = new SpikeProcessor();

        // Init AprilTag Processors
        AprilTagProcessor tagProcessor1 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(974.173, 974.173,277.586,251.531)
                .build();

        AprilTagProcessor tagProcessor2 = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(825.850,825.850,326.316,244.585)
                .build();

        // Init Multi VisionPortal
        VisionPortal visionPortal1 = new VisionPortal.Builder()
                .addProcessor(tagProcessor1)
                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(portal1ViewID)
                .build();

        VisionPortal visionPortal2 = new VisionPortal.Builder()
                .addProcessor(tagProcessor2)
                .addProcessor(tapeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setCameraResolution(new Size(640, 480))
                .setLiveViewContainerId(portal2ViewID)
                .build();
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);


        double multiplier =0.75;
        boolean xReached = false;


        while (opModeInInit()) {
            int curPos = tapeProcessor.yTapeBottomEdge;
            telemetry.addData("curPos", curPos);
            telemetry.update();
        }
        drive.setPoseEstimate(new Pose2d(0 ,0, 0));

        if (opModeIsActive()){
            tapeAlign(tapeProcessor, drive);



        }
    }
}
