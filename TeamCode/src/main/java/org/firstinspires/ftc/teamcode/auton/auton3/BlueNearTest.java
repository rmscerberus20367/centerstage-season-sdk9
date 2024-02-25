package org.firstinspires.ftc.teamcode.auton.auton3;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotLocalization;
import org.firstinspires.ftc.teamcode.SpikeProcessor;
import org.firstinspires.ftc.teamcode.auton.auton4.far0;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.function.ArmWrist;
import org.firstinspires.ftc.teamcode.function.Grip;
import org.firstinspires.ftc.teamcode.function.Slides;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
@Disabled
public class BlueNearTest extends LinearOpMode {
    Grip claw = new Grip();
    ArmWrist arm = new ArmWrist();
    Slides slides = new Slides();

    Pose2d startPose = new Pose2d(-36, -64, Math.toRadians(90));
    ElapsedTime timer = new ElapsedTime();
    int targetTagID;
    TrajectorySequence traj1left, traj1center, traj1right, traj2left, traj2center, traj2right, traj3, traj3center, traj3left, traj3right;

    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);
        arm.init(hardwareMap);
        slides.init(hardwareMap);
        SpikeProcessor spikeProcessor = new SpikeProcessor();
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(974.173, 974.173,277.586,251.531)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .addProcessor(spikeProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        claw.closeLeft();
        claw.closeRight();
        claw.update();
        int x = 6;
        int yMultipiler = -1;
        int headingSubtractor = 180;
        Pose2d startPose = new Pose2d(-36, 64 * yMultipiler, Math.toRadians(270 - headingSubtractor));
        drive.setPoseEstimate(startPose);
        traj1center = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(3)
                .addDisplacementMarker(() -> {
                    arm.intakePos();
                    arm.update();
                })
                .forward(24)
                .build();
        traj2center = drive.trajectorySequenceBuilder(traj1center.end())
                .back(12)
                .lineToLinearHeading(new Pose2d(-36, 62 * yMultipiler, Math.toRadians(180)))
                .build();


        traj1left = drive.trajectorySequenceBuilder(startPose)



                .lineToLinearHeading(new Pose2d(-48, 33 * yMultipiler, Math.toRadians(0)))
                .addDisplacementMarker(30, () -> {
                    arm.intakePos();
                    arm.update();
                })
                .forward(8)
                .build();
        traj2left = drive.trajectorySequenceBuilder(traj1left.end())
                .back(12)
                .lineToLinearHeading(new Pose2d(-36, 62 * yMultipiler, Math.toRadians(180)))
                .build();


        traj1right = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(3)
                .lineToLinearHeading(new Pose2d(-48, 48 * yMultipiler, Math.toRadians(270-headingSubtractor)))
                .addDisplacementMarker(5, () -> {
                    arm.intakePos();
                    arm.update();

                })
                .build();
        traj2right = drive.trajectorySequenceBuilder(traj1right.end())
                .back(12)


                .lineToLinearHeading(new Pose2d(-36, 62 * yMultipiler, Math.toRadians(180)))


                .build();
        traj3 = drive.trajectorySequenceBuilder(traj2center.end())
                .waitSeconds(1)
                .back(72)
                .lineTo(new Vector2d(38, 36*yMultipiler))
                .build();
        traj3left = drive.trajectorySequenceBuilder(traj3.end())
                .back(5)
                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1-100;
                    slides.update();
                    arm.update();


                    //timer.reset();
                    //while (timer.seconds()<1){}
                })
                .lineToLinearHeading(new Pose2d(48, (39+(x*7/6))*yMultipiler, Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1-100;
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
        traj3right = drive.trajectorySequenceBuilder(traj3.end())


                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1-100;
                    slides.update();
                    arm.update();
                    claw.closeLeft();
                    claw.update();

                    //timer.reset();
                    //while (timer.seconds()<1){}
                })
                .lineToLinearHeading(new Pose2d(48, (26+x)*yMultipiler, Math.toRadians(180)))

                .back(4)
                .waitSeconds(1)


                .build();
        traj3center = drive.trajectorySequenceBuilder(traj3.end())

                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1-100;
                    slides.update();
                    arm.update();


                    //timer.reset();
                    //while (timer.seconds()<1){}
                })
                .lineToLinearHeading(new Pose2d(48, (31.5+x)*yMultipiler, Math.toRadians(180)))

                .addDisplacementMarker(()->{
                    arm.depositPos();
                    slides.slidePos = slides.pos1-100;
                    slides.update();
                    arm.update();
                    timer.reset();

                })
                .back(4)
                .waitSeconds(1)


                .build();
        claw.init(hardwareMap);
        slides.init(hardwareMap);
        arm.init(hardwareMap);
        claw.closeLeft();
        claw.closeRight();
        claw.update();
        waitForStart();
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(180)));
        if (opModeIsActive()){
            RobotLocalization.getRobotPose(tagProcessor, drive);
            drive.update();
            telemetry.addData("x",drive.getPoseEstimate());
            telemetry.update();
            drive.followTrajectorySequence(traj3left);
            while (opModeIsActive()){}


        }
    }
}
