package org.firstinspires.ftc.teamcode.ftclib.extra_classes;



import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.ftclib.BotHardware;

public class TeleOp extends CommandOpMode {
    private GripperSubsystem gripper;
    BotHardware robit;
    private SimplifiedDriveSubsystem drive;
    private SimplifiedDriveCommand driveCommand;

    @Override
    public void initialize() {



        drive = new SimplifiedDriveSubsystem(robit.drive, robit.driverOp, robit.imu);
        driveCommand = new SimplifiedDriveCommand(drive, robit.driverOp, robit.imu);

        // using InstantCommand here is not the greatest idea because the servos move in nonzero time
        // alternatives are adding WaitUntilCommands or making these commands.
        // As a result of this uncertainty, we add the gripper subsystem to ensure requirements are met.
        drive.setDefaultCommand(driveCommand);
    }
}
