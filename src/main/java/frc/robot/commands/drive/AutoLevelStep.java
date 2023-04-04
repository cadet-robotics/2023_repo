package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelStep extends CommandBase {
    private RobotContainer robotContainer;
    
    private int stepLength;
    private int elapsed = 0;
    
    private double speed;

    public AutoLevelStep(RobotContainer robotContainer, int stepLength) {
        this.robotContainer = robotContainer;
        this.stepLength = stepLength;
    }

    @Override
    public void initialize() {
        double percentLevel = getAdjustedPitch() / DriveConstants.MAX_PITCH;
        speed = percentLevel * DriveConstants.AUTO_LEVEL_SPEED;
        robotContainer.driveSubsystem.drive(
            0, 
            speed, 
            0, 
            true, 
            false
        );
    }

    @Override
    public void execute() {
        elapsed++;
    }

    private double getAdjustedPitch() {
        return robotContainer.driveSubsystem.ahrs.getPitch() - DriveConstants.GYRO_PITCH;
    }

    @Override
    public boolean isFinished() {
        return elapsed >= stepLength;
    }

    @Override
    public void end(boolean interrupted) {
        robotContainer.driveSubsystem.setX();
    }
}
