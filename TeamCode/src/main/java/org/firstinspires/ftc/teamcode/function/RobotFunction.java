package org.firstinspires.ftc.teamcode.function;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotFunction {
    Grip claw  = new Grip();
    Drivetrain drive  = new Drivetrain();
    Slides slides  = new Slides();
    ArmWrist arm  = new ArmWrist();
    KeyReader lb = new KeyReader();
    KeyReader rb = new KeyReader();
    KeyReader rs = new KeyReader();
    KeyReader ls = new KeyReader();
    KeyReader b = new KeyReader();
    TriggerReader lt = new TriggerReader();
    TriggerReader rt = new TriggerReader();
    public int slidePos = 0;

    public void init(HardwareMap hardwareMap){
        claw.init(hardwareMap);
        drive.init(hardwareMap);
        slides.init(hardwareMap);
        arm.init(hardwareMap);
    }
    public void update(Gamepad gamepad1, Gamepad gamepad2){

        if (lb.wasJustPressed(gamepad2.left_bumper)){
            claw.toggleLeft();
        }
        if (rb.wasJustPressed(gamepad2.right_bumper)){
            claw.toggleRight();
        }
        if (rs.wasJustPressed(gamepad2.right_stick_button)){
            claw.toggleStates();
        }
        if (lt.wasJustPressed(gamepad2.left_trigger)){
            if (arm.leftArmPos == arm.leftArmUp){
                switch (slidePos){

                    case 1:
                        slides.slidesPos2();
                        slidePos = 2;
                        break;
                    case 2:
                        slides.slidesPos3();
                        slidePos = 3;
                        break;
                }
            }
            else {
                slides.slidesPos1();
                slidePos = 1;
                arm.depositPos();
            }
        }
        if (rt.wasJustPressed(gamepad2.right_trigger)){
            switch (slidePos){
                case 3:
                    slides.slidesPos2();
                    slidePos = 2;
                    break;
                case 2:
                    slides.slidesPos1();
                    slidePos = 1;
                    break;

            }
        }
        if (ls.wasJustPressed(gamepad2.left_stick_button)){
            arm.drivePos();
            slides.slidesDown();
            slidePos = 0;
        }
        if (gamepad2.a){
            arm.intakePos();
        }
        if (gamepad2.dpad_up){
            slides.manualUp();
        }
        if (gamepad2.dpad_down){
            slides.manualDown();
        }
        arm.update();
        drive.update(gamepad1);
        slides.update();
        claw.update();
    }

}
