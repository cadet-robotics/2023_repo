package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelCycle extends CommandBase {
    private RobotContainer robotContainer;

    private RepeatCommand stepper;

    public AutoLevelCycle(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
    }

    @Override
    public void initialize() {
        stepper = new AutoLevelStep(robotContainer, 5).repeatedly();
        stepper.schedule();
    }

    @Override
    public boolean isFinished() {
        // implements angular margin
        double pitch = robotContainer.driveSubsystem.ahrs.getPitch() - DriveConstants.GYRO_PITCH;
        SmartDashboard.putBoolean(
            "autoLevelNew/isFinished", 
            pitch < DriveConstants.AUTO_LEVEL_ANGULAR_MARGIN && pitch > -DriveConstants.AUTO_LEVEL_ANGULAR_MARGIN
        );
        return pitch < DriveConstants.AUTO_LEVEL_ANGULAR_MARGIN && pitch > -DriveConstants.AUTO_LEVEL_ANGULAR_MARGIN;
    }

    @Override
    public void end(boolean interrupted) {
        stepper.cancel();
    }
}
