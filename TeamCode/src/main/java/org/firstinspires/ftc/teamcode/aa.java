package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

/**
 * Example opmode demonstrating how to break out from a live trajectory at any arbitrary point in
 * time. This will allow you to do some cool things like incorporating live trajectory following in
 * your teleop. Check out TeleOpAgumentedDriving.java for an example of such behavior.
 * <p>
 * 3 seconds into the start of the opmode, `drive.breakFollowing()` is called, breaking out of all
 * trajectory following.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java and TrajectorySequenceRunnerCancelable.java
 * classes. Please ensure that these files are copied into your own project.
 */
@TeleOp
public class aa extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        // Set the pose estimate to where you know the bot will start in autonomous
        // Refer to https://www.learnroadrunner.com/trajectories.html#coordinate-system for a map
        // of the field
        // This example sets the bot at x: -20, y: -35, and facing 90 degrees (turned counter-clockwise)
        Pose2d startPose = new Pose2d(-20, -35, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        // Example spline path from SplineTest.java
        Trajectory traj= null;

        // We follow the trajectory asynchronously so we can run our own logic


        // Start the timer so we know when to cancel the following
        ElapsedTime stopTimer = new ElapsedTime();
        int x =78;
        if (opModeIsActive()) {
            if (x>0){
                traj = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(12)
                        .build();
            }
            while (opModeIsActive() && !isStopRequested()) {




            }
        }
    }
}