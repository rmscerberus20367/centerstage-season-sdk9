package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.function.Grip;
import org.firstinspires.ftc.teamcode.function.TriggerReader;

@TeleOp

public class GripTest extends LinearOpMode {
    Grip claw = new Grip();
    TriggerReader Rs = new TriggerReader();
    double b;
    @Override
    public void runOpMode() throws InterruptedException {
        claw.init(hardwareMap);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()){
            Boolean a = Rs.wasJustPressed(gamepad1.left_trigger);
            if (a){
                claw.toggleStates();
                claw.update();
                b++;
            }
            telemetry.addData("leftPos", claw.leftPos);
            telemetry.addData("b", b);

            telemetry.addData("right stick", a);
            telemetry.update();

        }
    }
}
