package org.firstinspires.ftc.teamcode.ftclib.extra_classes;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;

public class SimplifiedDriveSubsystem extends SubsystemBase {
    MecanumDrive drive;
    GamepadEx gamepad;
    RevIMU imu;
    public SimplifiedDriveSubsystem(final MecanumDrive Drive, final GamepadEx Gamepad, final RevIMU Imu) {
        drive = Drive;
        gamepad = Gamepad;
        imu = Imu;
    }
    public void driveRobotCentric(GamepadEx Gamepad){
        gamepad=Gamepad;
        drive.driveRobotCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX());
    }
    public void driveFieldCentric(GamepadEx gamepad, RevIMU imu){
        drive.driveFieldCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightX(), imu.getHeading());
    }
}
