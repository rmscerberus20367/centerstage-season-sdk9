package org.firstinspires.ftc.teamcode.ftclib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftclib.extra_classes.SimplifiedDrive;


public class BotRun extends LinearOpMode {

    BotHardware robit;
    SimplifiedDrive drive;
    public void runOpMode() {
        robit.init(hardwareMap);

        drive.setDrive(robit.drive);

        waitForStart();
        while (opModeIsActive()) {
            drive.driveRobotCentric(robit.driverOp);
            drive.driveFieldCentric(robit.driverOp, robit.imu);
        }
    }
}