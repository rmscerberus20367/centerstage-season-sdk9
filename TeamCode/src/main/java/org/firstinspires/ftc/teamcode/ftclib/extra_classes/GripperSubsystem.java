package org.firstinspires.ftc.teamcode.ftclib.extra_classes;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GripperSubsystem extends SubsystemBase {
    private final Servo gripper;

    public GripperSubsystem(final HardwareMap hMap, final String name) {
        gripper = hMap.get(Servo.class, name);
    }

    /**
     * Grabs a stone.
     */
    public void grab() {
        gripper.setPosition(0.76);
    }

    /**
     * Releases a stone.
     */
    public void release() {
        gripper.setPosition(0);
    }

}
