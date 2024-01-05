package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.function.Drivetrain;
import org.firstinspires.ftc.teamcode.function.RobotFunction;

@TeleOp
public class Pumpkin extends LinearOpMode {
    RobotFunction robot  = new RobotFunction();



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            robot.update(gamepad1, gamepad2);
            telemetry.addData("data", robot.slidePos);
            telemetry.update();
        }
    }
}
