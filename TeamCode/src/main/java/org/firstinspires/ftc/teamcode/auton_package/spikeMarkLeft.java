package org.firstinspires.ftc.teamcode.auton_package;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class spikeMarkLeft {
    SampleMecanumDrive drive;
    Pose2d startPose;
    public void set(Pose2d startPose1, SampleMecanumDrive drive1){
        drive = drive1;
        startPose = startPose1;

    }

    //Pose2d startPose = new Pose2d(12, 60, Math.toRadians(90));
    Pose2d currentPose = startPose;
    public void spikeMarkLeft(Pose2d currentPose1, int targetTagId){
        currentPose = currentPose1;
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

    }
}
