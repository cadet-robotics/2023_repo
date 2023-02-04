package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetWheelLockStateCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private boolean lockedState;

    public SetWheelLockStateCommand(DriveSubsystem driveSubsystem, boolean lockedState) {
        this.driveSubsystem = driveSubsystem;
        this.lockedState = lockedState;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.printf("[WheelLockState] Set to %s\n", lockedState);
        driveSubsystem.setDriveEnabled(!lockedState);
        if (lockedState) {
            driveSubsystem.setX();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
