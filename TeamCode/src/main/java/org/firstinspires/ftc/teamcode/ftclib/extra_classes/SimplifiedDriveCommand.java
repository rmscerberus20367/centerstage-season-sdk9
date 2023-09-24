package org.firstinspires.ftc.teamcode.ftclib.extra_classes;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;

public class SimplifiedDriveCommand extends CommandBase {


    private final SimplifiedDriveSubsystem m_driveSubsystem;

    GamepadEx gamepad;
    RevIMU imu;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SimplifiedDriveCommand(SimplifiedDriveSubsystem subsystem, GamepadEx Gamepad, RevIMU Imu) {
        m_driveSubsystem = subsystem;
        gamepad = Gamepad;
        imu = Imu;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        m_driveSubsystem.driveRobotCentric(gamepad);
    }
}
