package frc.robot.commands.homing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetEncodersCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    public ResetEncodersCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        driveSubsystem.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
