package frc.robot.commands.homing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetHomingModeCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private boolean enableHoming;

    public SetHomingModeCommand(DriveSubsystem driveSubsystem, boolean enableHoming) {
        this.driveSubsystem = driveSubsystem;
        this.enableHoming = enableHoming;
        
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.setDriveEnabled(!enableHoming);
        driveSubsystem.homingMode = enableHoming;
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
