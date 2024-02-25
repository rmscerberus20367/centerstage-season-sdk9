package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.function.Drivetrain;
import org.firstinspires.ftc.teamcode.function.RobotFunction;

@TeleOp
public class Pumpkin extends LinearOpMode {
    RobotFunction robot  = new RobotFunction();
    Servo drone;



    @Override
    public void runOpMode() throws InterruptedException {
        drone = hardwareMap.servo.get("drone");
        robot.init(hardwareMap);
        drone.setPosition(1);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            robot.update(gamepad1, gamepad2);
            telemetry.addData("dat", robot.slidePos);
            if (gamepad1.a){
                drone.setPosition(1);
            }
            if (gamepad1.b){
                drone.setPosition(0.5);
            }

            telemetry.update();
        }
    }
}
