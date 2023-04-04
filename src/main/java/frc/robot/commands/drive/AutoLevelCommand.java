package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private int levelTicks = 0;
    private boolean inverse;

    public AutoLevelCommand(DriveSubsystem driveSubsystem, boolean inverse) {
        this.driveSubsystem = driveSubsystem;
        this.inverse = inverse;
    }

    @Override
    public void initialize() {
        levelTicks = 0;
        driveSubsystem.setDriveEnabled(false);
    }

    private void drive(double speed) {
        driveSubsystem.drive(
            speed * (inverse ? -1 : 1) * -1,
            0,
            0,
            true,
            false,
            true
        );
    }

    @Override
    public void execute()
    {
        double speed = 0;
        
        // get current pitch and check if level
        float curPitch = driveSubsystem.ahrs.getPitch() - DriveConstants.GYRO_PITCH;
        if(curPitch < DriveConstants.AUTO_LEVEL_ANGULAR_MARGIN + DriveConstants.AUTO_LEVEL_GOAL_PITCH &&
        curPitch > -DriveConstants.AUTO_LEVEL_ANGULAR_MARGIN + DriveConstants.AUTO_LEVEL_GOAL_PITCH)
        {
            // will trip isFinished()
            levelTicks++;
        }
        else
        {
            // not level, move the bot
            boolean fineSpeed = curPitch < DriveConstants.AUTO_LEVEL_FINE_ANGLE && curPitch > -DriveConstants.AUTO_LEVEL_FINE_ANGLE;
            speed = (fineSpeed ? DriveConstants.AUTO_LEVEL_FINE_SPEED : DriveConstants.AUTO_LEVEL_COARSE_SPEED) * (curPitch/30);
            drive(speed);

            levelTicks = 0;
            SmartDashboard.putBoolean("autolevel/fineSpeed", fineSpeed);
        }

        // debug data
        SmartDashboard.putNumber("autolevel/speed", speed);
        SmartDashboard.putNumber("autolevel/adjustedangle", curPitch);
        SmartDashboard.putNumber("autolevel/angle", driveSubsystem.ahrs.getPitch());

        /*LogManager.GYRO_PITCH_LOG.append(driveSubsystem.ahrs.getPitch());
        LogManager.GYRO_ADJUSTED_PITCH_LOG.append(curPitch);
        LogManager.AUTO_LEVEL_SPEED_LOG.append(speed);*/
    }

    @Override
    public boolean isFinished()
    {
        return levelTicks >= DriveConstants.AUTO_LEVEL_TICK_DELAY;
    }

    @Override
    public void end(boolean interrupted) {
        //drive(0);
        driveSubsystem.setX();
    }
}
