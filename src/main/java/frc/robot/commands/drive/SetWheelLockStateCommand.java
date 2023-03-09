package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetWheelLockStateCommand extends CommandBase {
    private RobotContainer robotContainer;
    private boolean lockedState;

    public SetWheelLockStateCommand(RobotContainer robotContainer, boolean lockedState) {
        this.robotContainer = robotContainer;
        this.lockedState = lockedState;
    }

    @Override
    public void initialize() {
        System.out.printf("[WheelLockState] Set to %s\n", lockedState);
        robotContainer.driveSubsystem.setDriveEnabled(!lockedState);
        if (lockedState) {
            robotContainer.driveSubsystem.setX();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
