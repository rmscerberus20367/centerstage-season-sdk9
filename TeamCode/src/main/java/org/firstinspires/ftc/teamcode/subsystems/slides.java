package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;

public class slides {
    DcMotor left;
    DcMotor right;
    GamepadEx gamepad;
    int setline;
    int setline1;
    int setline2;
    int setline3;
    public void newSlides(GamepadEx g, DcMotor l, DcMotor r){
        gamepad = g;
        left = l;
        right = r;
    }
    public void runSlides(){
        if (gamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            if (setline == 3) slides3();
            else if (setline == 2) slides2();
            else if (setline == 1) slides1();
        }
        else if (gamepad.getButton(GamepadKeys.Button.DPAD_UP)) slidesDown();
        if (gamepad.wasJustReleased(GamepadKeys.Button.Y) && setline <3){
            setline++;
        }
        else if (gamepad.wasJustReleased(GamepadKeys.Button.A) && setline >1){
            setline--;
        }
        if (gamepad.getLeftY()>0){
            manualSlides();
        }
    }
    public void setSetLine(int num){
        setline = num;
    }
    private void setPositionSlides(int position, double power){
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setTargetPosition(position);
        right.setTargetPosition(position);
        left.setPower(power);
        right.setPower(power);
    }
    public void slides3(){
        setPositionSlides(setline3, 0.5);
    }
    public void slides2(){
        setPositionSlides(setline2, 0.5);
    }
    public void slides1(){
        setPositionSlides(setline1, 0.5);
    }
    public void slidesDown(){
        setPositionSlides(0, 0.5);
    }
    public void manualSlides(){
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setPower(gamepad.getLeftY());
        right.setPower(gamepad.getLeftY());
    }

}
