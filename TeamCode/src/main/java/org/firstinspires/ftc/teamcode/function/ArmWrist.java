package org.firstinspires.ftc.teamcode.function;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmWrist {
    ServoEx armLeft;
    ServoEx armRight;
    ServoEx wrist;

    public double leftArmUp = 0.75;
    double rightArmUp = 0.25;
    double leftArmDown = 0.27;
    double rightArmDown = 0.73;

    double wristGround = 0.76;
    double wristDeposit = 0.25;
    double wristDrive = 0.14;
    double wristPos = wristDrive;
    public double leftArmPos = leftArmDown;
    double rightArmPos = rightArmDown;
    public void init(HardwareMap hardwareMap){
        armLeft = new SimpleServo(
                hardwareMap, "depositTiltLeft",  0, 300
        );
        armRight = new SimpleServo(
                hardwareMap, "depositTiltRight",  0, 300
        );
        wrist = new SimpleServo(
                hardwareMap, "intakeTilt",  0, 300
        );
    }

    public void wristGround(){
        wristPos = wristGround;
    }
    public void wristDrive(){
        wristPos = wristDrive;
    }
    public void wristDeposit(){
        wristPos = wristDeposit;
    }
    public void armUp(){
        leftArmPos = leftArmUp;
        rightArmPos = rightArmUp;
    }
    public void armDown(){
        leftArmPos = leftArmDown;
        rightArmPos = rightArmDown;
    }
    public void depositPos(){
        armUp();
        wristDeposit();
    }
    public void intakePos(){
        armDown();
        wristGround();
    }
    public void drivePos(){
        armDown();
        wristDrive();
    }
    public void update(){
        wrist.setPosition(wristPos);
        armRight.setPosition(rightArmPos);
        armLeft.setPosition(leftArmPos);
    }
}
