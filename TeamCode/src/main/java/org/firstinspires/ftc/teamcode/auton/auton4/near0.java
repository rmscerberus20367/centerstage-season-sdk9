package org.firstinspires.ftc.teamcode.auton.auton4;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLocalization;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.function.ArmWrist;
import org.firstinspires.ftc.teamcode.function.Grip;
import org.firstinspires.ftc.teamcode.function.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class near0 {
    SampleMecanumDrive drive;
    Grip claw  = new Grip();
    Slides slides  = new Slides();
    ArmWrist arm  = new ArmWrist();
    ElapsedTime timer = new ElapsedTime();
    AprilTagProcessor tagProcessor;

    int yMultipiler = 1;
    int headingSubtractor = 0;
    HardwareMap x;
    RevBlinkinLedDriver blinkin;
    TrajectorySequence traj1left, traj1center, traj1right, traj2left, traj2center, traj2right, traj3center, traj3left, traj3right;


    public enum Alliance {
        RED,
        BLUE
    }

    int back1 = 5;

    //traj1center



    //On blue : traj1right On red traj1left

    //On blue : traj1left On red traj1right
    public void init(SampleMecanumDrive drive1, HardwareMap hardwareMap, AprilTagProcessor tagProcessor1){
        drive = drive1;
        claw.init(hardwareMap);
        slides.init(hardwareMap);
        arm.init(hardwareMap);
        slides.reset();
        tagProcessor = tagProcessor1;
        x = hardwareMap;
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


    }
    public void create(Alliance alliance){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);


        claw.closeLeft();
        claw.closeRight();
        claw.update();
        if (alliance == Alliance.RED){
            yMultipiler = -1;
            headingSubtractor = 180;
        }
        int x = 0;
        if (yMultipiler == -1){
            x =4;
        }
        Pose2d startPose = new Pose2d(12, 64*yMultipiler, Math.toRadians(90+headingSubtractor));
        drive.setPoseEstimate(startPose);
        traj1center = drive.trajectorySequenceBuilder(startPose)
                .back(back1)
                .lineToLinearHeading(new Pose2d(28, (25.5-x)*yMultipiler, Math.toRadians(180)))
                .addDisplacementMarker(5, () -> {
                    arm.intakePos();
                    arm.update();
                })
                .build();
        traj2center = drive.trajectorySequenceBuilder(traj1center.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(37, 30*yMultipiler, Math.toRadians(180)))
                .forward(5)
                .build();


        traj3center = drive.trajectorySequenceBuilder(traj2center.end())

                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1-250;
                    slides.update();
                    arm.update();


                    //timer.reset();
                    //while (timer.seconds()<1){}
                })
                .lineToLinearHeading(new Pose2d(48, (34+x)*yMultipiler, Math.toRadians(180)))

                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1-250;
                    slides.update();
                    arm.update();
                    timer.reset();

                })
                .back(4)
                .waitSeconds(1)


                .build();
        traj1left = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .lineToLinearHeading(new Pose2d(37, 30*yMultipiler, Math.toRadians(180)))
                .addDisplacementMarker(5, () -> {
                    arm.intakePos();
                    arm.update();
                })
                .build();

        traj3left = drive.trajectorySequenceBuilder(traj1left.end())
                .back(5)
                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1-250;
                    slides.update();
                    arm.update();


                    //timer.reset();
                    //while (timer.seconds()<1){}
                })
                .lineToLinearHeading(new Pose2d(48, (40+x)*yMultipiler, Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1-250;
                    slides.update();
                    arm.update();
                    claw.closeLeft();
                    claw.update();
                    timer.reset();
                    //while (timer.seconds()<1){}
                })
                .back(4)
                .waitSeconds(1)


                .build();
        traj1right = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .lineToLinearHeading(new Pose2d(24, 33*yMultipiler, Math.toRadians(180)))
                .addDisplacementMarker(5, () -> {
                    arm.intakePos();
                    arm.update();

                })
                .forward(10)
                .build();
        traj2right  = drive.trajectorySequenceBuilder(traj1right.end())

                .back(11)
                .build();
        traj3right = drive.trajectorySequenceBuilder(traj2right.end())


                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1-250;
                    slides.update();
                    arm.update();
                    claw.closeLeft();
                    claw.update();

                    //timer.reset();
                    //while (timer.seconds()<1){}
                })
                .lineToLinearHeading(new Pose2d(48, (28+x)*yMultipiler, Math.toRadians(180)))

                .back(4)
                .waitSeconds(1)


                .build();

    }
    public void run(int targetTagID){
        if (yMultipiler == 1 ) {
            switch (targetTagID) {
                case 1:
                    drive.followTrajectorySequence(traj1left);

                    break;
                case 2:
                    drive.followTrajectorySequence(traj1center);

                    break;
                case 3:
                    drive.followTrajectorySequence(traj1right);

                    break;
            }
        }
        else {
            switch (targetTagID) {
                case 3:
                    drive.followTrajectorySequence(traj1left);

                    break;
                case 2:
                    drive.followTrajectorySequence(traj1center);

                    break;
                case 1:
                    drive.followTrajectorySequence(traj1right);

                    break;
            }
        }
        claw.openLeft();
        claw.update();
        timer.reset();
        while (timer.seconds()<0.3){}
        if (yMultipiler == 1 ) {
            switch (targetTagID) {

                case 2:
                    drive.followTrajectorySequence(traj2center);

                    break;
                case 3:
                    drive.followTrajectorySequence(traj2right);

                    break;
            }
        }
        else {
            switch (targetTagID) {
                case 2:
                    drive.followTrajectorySequence(traj2center);

                    break;
                case 1:
                    drive.followTrajectorySequence(traj2right);

                    break;
            }
        }

        RobotLocalization.getRobotPose(tagProcessor, drive);

        drive.update();

        if (yMultipiler == 1 ) {
            switch (targetTagID) {
                case 1:
                    drive.followTrajectorySequence(traj3left);

                    break;
                case 2:
                    drive.followTrajectorySequence(traj3center);

                    break;
                case 3:
                    drive.followTrajectorySequence(traj3right);

                    break;
            }
        }
        else {
            switch (targetTagID) {
                case 3:
                    drive.followTrajectorySequence(traj3left);

                    break;
                case 2:
                    drive.followTrajectorySequence(traj3center);

                    break;
                case 1:
                    drive.followTrajectorySequence(traj3right);

                    break;
            }
        }




        timer.reset();
        while (timer.seconds()<0.2){}
        claw.openRight();
        claw.update();
        timer.reset();
        while (timer.seconds()<0.7){}
        timer.reset();
        slides.slidePos = slides.pos2;
        slides.update();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(3)
                .addDisplacementMarker(()->{
                    slides.slidesDown();
                    arm.drivePos();
                    claw.openLeft();
                    claw.update();
                    slides.update();
                    arm.update();
                })
                .lineToLinearHeading(new Pose2d(48, 62*yMultipiler, Math.toRadians(180)))
                .back(12)
                .build();
        drive.followTrajectorySequence(traj2);



    }


}




