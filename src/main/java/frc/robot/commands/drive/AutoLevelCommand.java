package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;

    public AutoLevelCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute() {
        double speed = DriveConstants.AUTO_LEVEL_APPROACH_SPEED * getAdjustedGyroAngle();

        driveSubsystem.drive(
            speed,
            0,
            0,
            false,
            false
        );

        SmartDashboard.putNumber("autolevel/speed", speed);
        SmartDashboard.putNumber("autolevel/adjustedangle", getAdjustedGyroAngle());
    }

    @Override
    public boolean isFinished() {
        return getAdjustedGyroAngle() + DriveConstants.AUTO_LEVEL_ANGULAR_MARGIN > 0 &&
            getAdjustedGyroAngle() - DriveConstants.AUTO_LEVEL_ANGULAR_MARGIN < 0;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setDriveEnabled(false);
        driveSubsystem.setX();
    }

    private double getAdjustedGyroAngle() {
        return (double)driveSubsystem.ahrs.getPitch() - DriveConstants.GYRO_PITCH;
    }
}
