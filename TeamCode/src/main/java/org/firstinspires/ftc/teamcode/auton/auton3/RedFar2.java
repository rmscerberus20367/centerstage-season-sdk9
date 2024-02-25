package org.firstinspires.ftc.teamcode.auton.auton3;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SpikeProcessor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.function.ArmWrist;
import org.firstinspires.ftc.teamcode.function.Grip;
import org.firstinspires.ftc.teamcode.function.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
@Disabled
public class RedFar2 extends LinearOpMode {
    Grip claw = new Grip();
    ArmWrist arm = new ArmWrist();
    Slides slides = new Slides();

    Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(270));
    ElapsedTime timer = new ElapsedTime();
    int delay = 6;
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
                .strafeRight(5)
                .back(51)
                .addDisplacementMarker(5, () -> {
                    arm.intakePos();
                    arm.update();
                })
                .build();
        TrajectorySequence traj2center = drive.trajectorySequenceBuilder(traj1center.end())
                .addDisplacementMarker( () -> {

                    arm.drivePos();
                    arm.update();
                    //timer.reset();
                    //while (timer.seconds()<1){}
                })

                .back(1)
                .addDisplacementMarker(() ->{
                    arm.drivePos();
                    arm.update();
                })
                .turn(Math.toRadians(-90))
                .waitSeconds(delay)
                .back(5)
                .back(72)
                .lineToLinearHeading(new Pose2d(45, -41.6, Math.toRadians(180)))

                .addDisplacementMarker(85, ()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1+324;
                    slides.update();
                    arm.update();
                    timer.reset();
                    //while (timer.seconds()<1){}
                })
                .back(3)







                .build();
        TrajectorySequence traj1right = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(7)
                .back(34)
                .turn(Math.toRadians(87))
                .forward(4.5)

                .addDisplacementMarker(10, () -> {
                    arm.intakePos();
                    arm.update();
                })
                .build();
        TrajectorySequence traj2right  = drive.trajectorySequenceBuilder(traj1right.end())
                .addDisplacementMarker( () -> {

                    arm.drivePos();
                    arm.update();
                    //timer.reset();
                    //while (timer.seconds()<1){}
                })
                .back(2)
                .turn(Math.toRadians(-87))
                .back(18)
                .turn(Math.toRadians(-90))
                .waitSeconds(delay)
                .back(5)
                .back(72)
                .lineToLinearHeading(new Pose2d(45, -46, Math.toRadians(180)))
                .addDisplacementMarker(85, ()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1+324;
                    slides.update();
                    arm.update();
                    claw.closeLeft();
                    claw.update();
                    timer.reset();
                    //while (timer.seconds()<1){}
                })
                .back(3)





                .build();
        TrajectorySequence traj1left = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(5)
                .back(43)
                .strafeRight(9)
                .addDisplacementMarker(5, () -> {
                    arm.intakePos();
                    arm.update();
                })
                .build();
        TrajectorySequence traj2left = drive.trajectorySequenceBuilder(traj1left.end())
                .addDisplacementMarker( () -> {

                    arm.drivePos();
                    arm.update();
                    //timer.reset();
                    //while (timer.seconds()<1){}
                })
                .back(9)
                .turn(Math.toRadians(-90))
                .waitSeconds(delay)
                .back(5)
                .back(72)
                .lineToLinearHeading(new Pose2d(45, -34, Math.toRadians(180)))
                .addDisplacementMarker(85, ()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1+324;
                    slides.update();
                    arm.update();
                    claw.closeLeft();
                    claw.update();
                    telemetry.addData("pose", drive.getPoseEstimate());
                    telemetry.update();
                    //timer.reset();
                    //while (timer.seconds()<1){}
                })
                .back(3)


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
        timer.reset();
            TrajectorySequence traj2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(5)
                    .addDisplacementMarker(2, ()->{
                        claw.openLeft();
                        claw.update();
                        slides.slidesDown();
                        slides.update();
                        arm.drivePos();
                        arm.update();


                    })
                    .build();
        claw.openRight();
        claw.update();
            timer.reset();
            while (timer.seconds()<0.7){}
            timer.reset();
            slides.slidePos = slides.pos2+30;
            slides.update();
            timer.reset();

        while (timer.seconds()<0.7){}
        drive.followTrajectorySequence(traj2);
        timer.reset();
        claw.openLeft();
        claw.update();
        slides.slidesDown();
        slides.update();
        arm.drivePos();
        arm.update();

        while (timer.seconds()<3){}







    }}
}

