package org.firstinspires.ftc.teamcode.ftclib.extra_classes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;

public class SimplifiedDrive {
    MecanumDrive drive;
    public void setDrive(MecanumDrive Drive) {
        drive = Drive;
    }
    public void driveRobotCentric(GamepadEx gamepad){
        drive.driveRobotCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX());
    }
    public void driveFieldCentric(GamepadEx gamepad, RevIMU imu){
        drive.driveFieldCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), imu.getHeading());
    }
}
