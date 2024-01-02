package org.firstinspires.ftc.teamcode.function;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {


    Motor frontLeft, frontRight, backLeft, backRight;
    MecanumDrive mecanum;
    public void init(HardwareMap hardwareMap){
        frontLeft = new Motor(hardwareMap, "leftFront");
        frontRight = new Motor(hardwareMap, "rightFront");
        backLeft = new Motor(hardwareMap, "leftBack");
        backRight = new Motor(hardwareMap, "rightBack");
        frontLeft.setInverted(true);
        backRight.setInverted(true);
        mecanum = new MecanumDrive(frontLeft, frontRight,
                backLeft, backRight);

    }

    public void update(Gamepad gamepadEx){
        mecanum.driveRobotCentric(gamepadEx.left_stick_x, gamepadEx.left_stick_y, gamepadEx.right_stick_x);
    }

}
