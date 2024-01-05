package org.firstinspires.ftc.teamcode.function;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {
    DcMotor leftSlide;
    DcMotor rightSlide;
    int groundPos = 0;
    int pos1 = 625;
    int pos2 = 1200;
    int pos3 = 1800;
    int climb = 0;
    int changeValue = 5;
    int slidePos = 0;
    public void init(HardwareMap hardwareMap){
        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        rightSlide = hardwareMap.dcMotor.get("rightSlide");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void slidesDown(){
        slidePos = groundPos;
    }
    public void slidesPos1(){
        slidePos = pos1;
    }
    public void slidesPos2(){
        slidePos = pos2;
    }
    public void slidesPos3(){
        slidePos = pos3;
    }
    public void climb(){
        slidePos = climb;
    }
    public void manualUp(){
        if (leftSlide.getCurrentPosition()<1990) slidePos = slidePos+changeValue;
    }
    public void manualDown(){
        if (leftSlide.getCurrentPosition()>10)slidePos = slidePos-changeValue;
    }


    public void update(){
        leftSlide.setTargetPosition(slidePos);
        rightSlide.setTargetPosition(slidePos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.75);
        rightSlide.setPower(0.75);
    }

}
