package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.function.Slides;

@TeleOp
public class slidesTest extends LinearOpMode {

    Slides slides = new Slides();
    @Override
    public void runOpMode() throws InterruptedException {
        slides.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                slides.slidesPos1();
            }
            if (gamepad1.b){
                slides.slidesPos2();
            }
            if (gamepad1.y){
                slides.slidesPos3();
            }
            if (gamepad1.x){
                slides.slidesDown();
            }
            slides.update();

        }
    }
}
