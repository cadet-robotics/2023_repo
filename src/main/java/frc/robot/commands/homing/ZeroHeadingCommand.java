package frc.robot.commands.homing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ZeroHeadingCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    public ZeroHeadingCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        driveSubsystem.zeroHeading();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
