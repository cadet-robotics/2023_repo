package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private boolean level = false;

    public AutoLevelCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void execute()
    {
        double speed = 0;

        // get current pitch and check if level
        float curPitch = driveSubsystem.ahrs.getPitch();
        if(curPitch < DriveConstants.AUTO_LEVEL_ANGULAR_MARGIN &&
           curPitch > -DriveConstants.AUTO_LEVEL_ANGULAR_MARGIN)
        {
            // will trip isFinished()
            level = true;
        }
        else
        {
            // not level, move the bot
            speed = DriveConstants.AUTO_LEVEL_APPROACH_SPEED * curPitch;
            driveSubsystem.drive(
                0,
                speed,
                0,
                true,
                false
            );
        }

        SmartDashboard.putNumber("autolevel/speed", speed);
        SmartDashboard.putNumber("autolevel/adjustedangle", curPitch);
    }

    @Override
    public boolean isFinished()
    {
        return level;
    }
}
