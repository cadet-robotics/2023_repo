package frc.robot.commands.homing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetHomingModeCommand extends CommandBase {
    private RobotContainer robotContainer;
    private boolean enableHoming;

    public SetHomingModeCommand(RobotContainer robotContainer, boolean enableHoming) {
        this.robotContainer = robotContainer;
        this.enableHoming = enableHoming;
    }

    @Override
    public void initialize() {
        robotContainer.driveSubsystem.setDriveEnabled(!enableHoming);
        robotContainer.driveSubsystem.homingMode = enableHoming;
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
