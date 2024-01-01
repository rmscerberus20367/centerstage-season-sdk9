package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "🎃")
public class AmbroseTeleop_Copy extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftBack");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightBack");
        DcMotor leftSlide = hardwareMap.dcMotor.get("leftSlide");
        DcMotor rightSlide = hardwareMap.dcMotor.get("rightSlide");
        Servo gripTilt = hardwareMap.servo.get("intakeTilt");
        Servo depositTiltLeft = hardwareMap.servo.get("depositTiltLeft");
        Servo depositTiltRight = hardwareMap.servo.get("depositTiltRight");
        Servo gripLeft = hardwareMap.servo.get("gripLeft");
        Servo gripRight = hardwareMap.servo.get("gripRight");
        Servo drone  = hardwareMap.servo.get("gripRight");
        
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        drone.setPosition(0.5);
        
        
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x/3    ;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            
            rightSlide.setPower(-gamepad2.left_stick_y);
            leftSlide.setPower(-gamepad2.left_stick_y);
            if (gamepad2.a) {
                gripLeft.setPosition(0.5);
            }
            else if (gamepad2.b) {
                gripLeft.setPosition(0.3);
            }
             if (gamepad2.x) {
                gripRight.setPosition(0.5);
            }
            else if (gamepad2.y) {
                gripRight.setPosition(0.675);
            }
            if (gamepad2.dpad_up){
                depositTiltRight.setPosition(0);
                depositTiltLeft.setPosition(1);
            }
            if (gamepad2.dpad_down){
                depositTiltRight.setPosition(0.7);
                depositTiltLeft.setPosition(0.3);
            }
            if (gamepad2.dpad_left){
                gripTilt.setPosition(0);
            }
            if (gamepad2.dpad_right){
                gripTilt.setPosition(0.5);
            }
            if (gamepad1.x){
                drone.setPosition(1);
            }
            }
        }
    }

