package frc.robot.commands.homing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ZeroSwerveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    public ZeroSwerveCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        System.out.println("[WARN] Swerve has been zerod! Robot must be restarted to acknowledge offsets!");
        driveSubsystem.setModuleZeros();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}
