package org.firstinspires.ftc.teamcode.ftclib;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BotHardware {
    public Motor fl, fr, bl, br;
    public MecanumDrive drive;
    public GamepadEx driverOp;
    public Gamepad gamepad1;
    public RevIMU imu;

    HardwareMap hardwareMap = null;
    public void init(HardwareMap hardwareMap) {
        /* instantiate motors */

        fl = new Motor(hardwareMap, "frontLeft");
        fr = new Motor(hardwareMap, "frontRight");
        bl = new Motor(hardwareMap, "backLeft");
        br = new Motor(hardwareMap, "backRight");
        fl.setInverted(true);

        drive = new MecanumDrive(fl, fr, bl, br);

        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)

        imu = new RevIMU(hardwareMap);
        imu.init();

        driverOp = new GamepadEx(gamepad1);
    }
}
