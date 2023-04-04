package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToRampCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private boolean inverse;

    public DriveToRampCommand(DriveSubsystem driveSubsystem, boolean inverse) {
        this.driveSubsystem = driveSubsystem;
        this.inverse = inverse;
    }

    private void drive(double speed) {
        driveSubsystem.drive(
            speed * (inverse ? -1 : 1),
            0,
            0,
            true,
            false,
            true
        );
    }

    @Override
    public void initialize() {
        driveSubsystem.setDriveEnabled(false);
        drive(DriveConstants.AUTO_LEVEL_APPROACH_SPEED);
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.ahrs.getPitch() - DriveConstants.GYRO_PITCH > DriveConstants.AUTO_LEVEL_ENTRY_ANGLE ||
            driveSubsystem.ahrs.getPitch() - DriveConstants.GYRO_PITCH < -DriveConstants.AUTO_LEVEL_ENTRY_ANGLE;
    }

    @Override
    public void end(boolean interrupted) {
        drive(0);
    }
}
