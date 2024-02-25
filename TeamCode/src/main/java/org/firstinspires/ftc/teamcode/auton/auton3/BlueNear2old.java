package org.firstinspires.ftc.teamcode.auton.auton3;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RobotLocalization;
import org.firstinspires.ftc.teamcode.SpikeProcessor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.function.ArmWrist;
import org.firstinspires.ftc.teamcode.function.Grip;
import org.firstinspires.ftc.teamcode.function.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
@Disabled

public class BlueNear2old extends LinearOpMode {
    Grip claw = new Grip();
    ArmWrist arm = new ArmWrist();
    Slides slides = new Slides();

    Pose2d startPose = new Pose2d(12, 64, Math.toRadians(90));
    ElapsedTime timer = new ElapsedTime();
    int targetTagID;

    @Override
    public void runOpMode() throws InterruptedException {
        SpikeProcessor spikeProcessor = new SpikeProcessor();
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(974.173, 974.173,277.586,251.531)
                .build();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        TrajectorySequence traj1center = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .lineToLinearHeading(new Pose2d(28, 25.5, Math.toRadians(180)))
                .addDisplacementMarker(5, () -> {
                    arm.intakePos();
                    arm.update();
                })
                .build();
        TrajectorySequence traj2center = drive.trajectorySequenceBuilder(traj1center.end())
                
                .back(5)


                .lineToLinearHeading(new Pose2d(48, 36, Math.toRadians(180)))

                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1+30;
                    slides.update();
                    arm.update();
                    timer.reset();
                    //while (timer.seconds()<1){}
                })
                .back(3)


                .build();
        TrajectorySequence traj1left = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .lineToLinearHeading(new Pose2d(37, 30, Math.toRadians(180)))
                .addDisplacementMarker(5, () -> {
                    arm.intakePos();
                    arm.update();
                })
                .build();
        TrajectorySequence traj2left = drive.trajectorySequenceBuilder(traj1left.end())
                
                .back(5)


                .lineToLinearHeading(new Pose2d(48, 40, Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1+30;
                    slides.update();
                    arm.update();
                    claw.closeLeft();
                    claw.update();
                    timer.reset();
                    //while (timer.seconds()<1){}
                })
                .back(3)


                .build();
        TrajectorySequence traj1right = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .lineToLinearHeading(new Pose2d(24, 33, Math.toRadians(180)))
                .addDisplacementMarker(5, () -> {
                    arm.intakePos();
                    arm.update();
                    telemetry.addData("pose", drive.getPoseEstimate());
                    telemetry.update();
                })
                .forward(10)
                .build();
        TrajectorySequence traj2right  = drive.trajectorySequenceBuilder(traj1right.end())
                
                .back(11)
                .build();
        TrajectorySequence traj3right = drive.trajectorySequenceBuilder(traj2right.end())


                .lineToLinearHeading(new Pose2d(48, 28, Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1+30;
                    slides.update();
                    arm.update();
                    claw.closeLeft();
                    claw.update();
                    telemetry.addData("pose", drive.getPoseEstimate());
                    telemetry.update();
                    //timer.reset();
                    //while (timer.seconds()<1){}
                })
                .back(4.5)


                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1center.end())
                .forward(3)
                .addDisplacementMarker(()->{
                    slides.slidesDown();
                    arm.drivePos();
                    claw.openLeft();
                    claw.update();
                    slides.update();
                    arm.update();
                })
                .strafeRight(24)
                .back(16)
                .build();
        drive.setPoseEstimate(startPose);
        claw.init(hardwareMap);
        slides.init(hardwareMap);
        arm.init(hardwareMap);
        claw.closeLeft();
        claw.closeRight();
        claw.update();
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

        if (opModeIsActive()){
            switch (targetTagID){
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
            claw.openLeft();
            claw.update();
            timer.reset();
            while (timer.seconds()<0.7){}
            switch (targetTagID){
                case 1:
                    drive.followTrajectorySequence(traj2left);

                    break;
                case 2:
                    drive.followTrajectorySequence(traj2center);

                    break;
                case 3:
                    drive.followTrajectorySequence(traj2right);

                    break;
            }

            telemetry.addData("x", RobotLocalization.getRobotPose(tagProcessor, drive));
            drive.update();
            telemetry.update();
            switch (targetTagID){
                /*case 1:
                    drive.followTrajectorySequence(traj2left);

                    break;
                case 2:
                    drive.followTrajectorySequence(traj2center);

                    break;*/
                case 3:
                    drive.followTrajectorySequence(traj3right);

                    break;
            }


            timer.reset();
            while (timer.seconds()<0.7){}
            claw.openRight();
            claw.update();
            timer.reset();
            while (timer.seconds()<0.7){}
            timer.reset();
            slides.slidePos = slides.pos2+30;
            slides.update();
            traj2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(3)
                    .addDisplacementMarker(()->{
                        slides.slidesDown();
                        arm.drivePos();
                        claw.openLeft();
                        claw.update();
                        slides.update();
                        arm.update();
                    })
                    .lineToLinearHeading(new Pose2d(51, 62, Math.toRadians(180)))
                    .back(12)
                    .build();


            while (timer.seconds()<0.5){}

            drive.followTrajectorySequence(traj2);





        }
    }
}
