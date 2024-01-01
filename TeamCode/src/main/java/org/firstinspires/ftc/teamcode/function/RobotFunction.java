package org.firstinspires.ftc.teamcode.function;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotFunction {
    Grip claw  = new Grip();
    Drivetrain drive  = new Drivetrain();
    Slides slides  = new Slides();
    ArmWrist arm  = new ArmWrist();
    int slidePos = 0;

    public void init(HardwareMap hardwareMap){
        claw.init(hardwareMap);
        drive.init(hardwareMap);
        slides.init(hardwareMap);
        arm.init(hardwareMap);
    }
    public void update(GamepadEx gamepad1, GamepadEx gamepad2){
        TriggerReader leftTrigger = new TriggerReader(
                gamepad2, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        TriggerReader rightTrigger = new TriggerReader(
                gamepad2, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        if (gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
            claw.switchStatesLeft();
        }
        if (gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
            claw.switchStatesRight();
        }
        if (gamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
            claw.switchStates();
        }
        if (leftTrigger.wasJustPressed()){
            if (arm.leftArmPos == arm.leftArmUp){
                switch (slidePos){
                    case 1:
                        slides.slidesPos2();
                        slidePos = 2;
                    case 2:
                        slides.slidesPos3();
                        slidePos = 3;
                }
            }
            else {
                slides.slidesPos1();
                slidePos = 1;
                arm.depositPos();
            }
        }
        if (rightTrigger.wasJustPressed()){
            switch (slidePos){
                case 3:
                    slides.slidesPos2();
                    slidePos = 2;
                case 2:
                    slides.slidesPos1();
                    slidePos = 1;
            }
        }
        if (gamepad2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            arm.drivePos();
        }
        if (gamepad2.wasJustPressed(GamepadKeys.Button.B)){
            arm.intakePos();
        }
        if (gamepad2.getButton(GamepadKeys.Button.DPAD_UP)){
            slides.manualUp();
        }
        if (gamepad2.getButton(GamepadKeys.Button.DPAD_DOWN)){
            slides.manualDown();
        }
        arm.update();
        drive.update(gamepad1);
        slides.update();
        claw.update();
    }

}
