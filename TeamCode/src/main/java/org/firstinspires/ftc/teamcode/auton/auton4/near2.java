package org.firstinspires.ftc.teamcode.auton.auton4;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotLocalization;
import org.firstinspires.ftc.teamcode.StackTapeDetection;
import org.firstinspires.ftc.teamcode.StackTapeProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.function.ArmWrist;
import org.firstinspires.ftc.teamcode.function.Grip;
import org.firstinspires.ftc.teamcode.function.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class near2 {
    SampleMecanumDriveCancelable drive;
    Grip claw  = new Grip();
    Slides slides  = new Slides();
    ArmWrist arm  = new ArmWrist();
    ElapsedTime timer = new ElapsedTime();
    AprilTagProcessor tagProcessor;
    StackTapeProcessor tapeProcessor;
    int yMultipiler = 1;
    int headingSubtractor = 0;
    near0 near0 = new near0();
    HardwareMap x;
    SampleMecanumDrive drive1;

    public enum Alliance {
        RED,
        BLUE
    }

    int back1 = 5;

    //traj1center



    //On blue : traj1right On red traj1left

    //On blue : traj1left On red traj1right
    public void init(HardwareMap hardwareMap, AprilTagProcessor tagProcessor1, StackTapeProcessor tapeProcessor1){
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        claw.init(hardwareMap);
        slides.init(hardwareMap);
        arm.init(hardwareMap);
        slides.reset();
        tagProcessor = tagProcessor1;
        x = hardwareMap;
        tapeProcessor = tapeProcessor1;
        drive1 = new SampleMecanumDrive(hardwareMap);
        near0.init(drive1, hardwareMap, tagProcessor1);


    }
    public void create(near0.Alliance alliance){

        near0.create(alliance);

        if (alliance == org.firstinspires.ftc.teamcode.auton.auton4.near0.Alliance.RED) {
            yMultipiler = -1;
            headingSubtractor = 180;
        }
        Pose2d startPose = new Pose2d(12, 64*yMultipiler, Math.toRadians(90+headingSubtractor));
        drive.setPoseEstimate(startPose);
    }
    public void run(int targetTagID){
        if (yMultipiler == 1 ) {
            switch (targetTagID) {
                case 1:
                    drive.followTrajectorySequence(near0.traj1left);

                    break;
                case 2:
                    drive.followTrajectorySequence(near0.traj1center);

                    break;
                case 3:
                    drive.followTrajectorySequence(near0.traj1right);

                    break;
            }
        }
        else {
            switch (targetTagID) {
                case 3:
                    drive.followTrajectorySequence(near0.traj1left);

                    break;
                case 2:
                    drive.followTrajectorySequence(near0.traj1center);

                    break;
                case 1:
                    drive.followTrajectorySequence(near0.traj1right);

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
                    drive.followTrajectorySequence(near0.traj2center);

                    break;
                case 3:
                    drive.followTrajectorySequence(near0.traj2right);

                    break;
            }
        }
        else {
            switch (targetTagID) {
                case 2:
                    drive.followTrajectorySequence(near0.traj2center);

                    break;
                case 1:
                    drive.followTrajectorySequence(near0.traj2right);

                    break;
            }
        }

        RobotLocalization.getRobotPose(tagProcessor, drive);

        drive.update();

        if (yMultipiler == 1 ) {
            switch (targetTagID) {
                case 1:
                    drive.followTrajectorySequence(near0.traj3left);

                    break;
                case 2:
                    drive.followTrajectorySequence(near0.traj3center);

                    break;
                case 3:
                    drive.followTrajectorySequence(near0.traj3right);

                    break;
            }
        }
        else {
            switch (targetTagID) {
                case 3:
                    drive.followTrajectorySequence(near0.traj3left);

                    break;
                case 2:
                    drive.followTrajectorySequence(near0.traj3center);

                    break;
                case 1:
                    drive.followTrajectorySequence(near0.traj3right);

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
                .lineToLinearHeading(new Pose2d(42.5, 12*yMultipiler, Math.toRadians(180)))


                .lineToLinearHeading(new Pose2d(-54, 12*yMultipiler, Math.toRadians(180)))
                .addDisplacementMarker(85, ()->{
                    arm.intakePos();
                    arm.update();
                    slides.slidePos = 220;
                    slides.update();

                    claw.update();
                })
                .build();


        while (timer.seconds()<0.5){}

        drive.followTrajectorySequence(traj2);

        StackTapeDetection.tapeAlign(tapeProcessor, drive);
        drive.update();

        TrajectorySequence traj4  = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeRight(5)
                .addDisplacementMarker(0, ()->{
                    arm.intakePos();
                    arm.update();


                    claw.update();
                })
                .waitSeconds(1)
                .forward(13)
                .build();
        drive.followTrajectorySequence(traj4);
        slides.slidePos = 210;
        //while (timer.seconds()<0.5){}
        claw.closeLeft();
        claw.update();
        timer.reset();
        while (timer.seconds()<0.7){}
        slides.slidePos = 355;
        slides.update();

        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(42.5, 10*yMultipiler, Math.toRadians(180)))
                .addDisplacementMarker(10, ()->{
                    slides.slidesDown();
                    arm.drivePos();
                    slides.update();
                    arm.update();
                })
                .addDisplacementMarker(()->{
            arm.depositPos();
            slides.slidesPos1();
            slides.update();
            arm.wristPos = arm.wristDeposit+0.1;
            arm.update();
            arm.update();
            claw.closeLeft();
            claw.update();

            //timer.reset();
            //while (timer.seconds()<1){}
                })
                .lineToLinearHeading(new Pose2d(48, 35*yMultipiler, Math.toRadians(180)))

                .back(0.5)
                .build();
        drive.followTrajectorySequence(traj5);

        claw.openLeft();
        claw.update();
        timer.reset();
        while (timer.seconds()<1){}


    }


}
