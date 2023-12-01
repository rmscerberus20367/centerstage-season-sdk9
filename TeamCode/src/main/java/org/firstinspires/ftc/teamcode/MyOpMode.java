
package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@TeleOp
public class MyOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                //.back(5)
                .lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(90)))
                .waitSeconds(3)
                .lineToLinearHeading(new Pose2d(48, -36, Math.toRadians(180)))
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                .build()


        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(traj);
    }
}