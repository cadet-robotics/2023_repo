package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class FinalizeLevelCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private int ticksElapsed = 0;
    private boolean inverse;

    public FinalizeLevelCommand(DriveSubsystem driveSubsystem, boolean inverse) {   
        this.driveSubsystem = driveSubsystem;
        this.inverse = inverse;
    }

    private void drive(double speed) {
        driveSubsystem.drive(speed, 0, 0, true, false, true);
    }
    
    @Override
    public void initialize() {
        ticksElapsed = 0;
        drive((inverse ? -1 : 1) * -DriveConstants.FINALIZE_LEVEL_SPEED);
    }

    @Override
    public void execute() {
        ticksElapsed++;
    }

    @Override
    public boolean isFinished() {
        return ticksElapsed >= DriveConstants.FINALIZE_LEVEL_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setX();
    }
}
