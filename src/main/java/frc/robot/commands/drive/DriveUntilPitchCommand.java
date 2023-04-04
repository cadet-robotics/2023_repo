package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveUntilPitchCommand extends CommandBase {
    private RobotContainer robotContainer;
    private float goalPitch;
    private double speed;

    public DriveUntilPitchCommand(RobotContainer robotContainer, float goalPitch, double speed) {
        this.robotContainer = robotContainer;
        this.goalPitch = goalPitch;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        // get direction
        int direction = getAdjustedPitch() > goalPitch ? -1 : 1;
        robotContainer.driveSubsystem.drive(
            0,
            speed * direction,
            0,
            true,
            false
        );
    }

    private float getAdjustedPitch() {
        return robotContainer.driveSubsystem.ahrs.getPitch() - DriveConstants.GYRO_PITCH;
    }

    @Override
    public boolean isFinished() {
        return getAdjustedPitch() < DriveConstants.DRIVE_UNTIL_PITCH_MARGIN &&
            getAdjustedPitch() > -DriveConstants.DRIVE_UNTIL_PITCH_MARGIN;
    }

    @Override
    public void end(boolean interrupted) {
        robotContainer.driveSubsystem.setX();
    }
}
