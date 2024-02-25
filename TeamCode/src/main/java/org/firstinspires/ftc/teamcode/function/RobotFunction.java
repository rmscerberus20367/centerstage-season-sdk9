package org.firstinspires.ftc.teamcode.function;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightBlinker;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotFunction {
    Grip claw  = new Grip();
    Drivetrain drive  = new Drivetrain();
    public Slides slides  = new Slides();
    ElapsedTime timer = new ElapsedTime();
    ArmWrist arm  = new ArmWrist();
    KeyReader lb = new KeyReader();
    KeyReader rb = new KeyReader();
    KeyReader rs = new KeyReader();
    KeyReader ls = new KeyReader();
    KeyReader b = new KeyReader();
    TriggerReader lt = new TriggerReader();
    TriggerReader rt = new TriggerReader();
    RevBlinkinLedDriver blinkin;
    public int slidePos = 0;

    public void init(HardwareMap hardwareMap){
        claw.init(hardwareMap);
        drive.init(hardwareMap);
        slides.init(hardwareMap);
        arm.init(hardwareMap);
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        timer.reset();
    }
    public void update(Gamepad gamepad1, Gamepad gamepad2){

        if (rb.wasJustPressed(gamepad2.right_bumper)){
            claw.toggleLeft();
        }
        if (lb.wasJustPressed(gamepad2.left_bumper)){
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
        if (gamepad2.b){
            arm.wristPos= arm.wristDeposit+0.1;
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

        if (claw.leftPos == claw.leftOpen && claw.rightPos == claw.rightOpen && slidePos > 0 || claw.leftPos == claw.leftClose && claw.rightPos == claw.rightClose && slidePos == 0 && arm.wristPos == arm.wristGround){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else if (claw.leftPos == claw.leftOpen && claw.rightPos == claw.rightClose || claw.leftPos == claw.leftClose && claw.rightPos == claw.rightOpen){
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        }
        else {
            if (timer.seconds()>90){
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
            }
            else {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
            }
        }


    }

}
