package frc.robot.commands.drive;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.RollingFloatAverager;

public class AutoRateLevelCommand extends CommandBase {
    private RobotContainer robotContainer;

    private RollingFloatAverager samples = new RollingFloatAverager(10); // TODO: tune this value for more accurate leveling
    private boolean isReversed;

    public AutoRateLevelCommand(RobotContainer robotContainer, boolean isReversed) {
        this.robotContainer = robotContainer;
        this.isReversed = isReversed;
    }

    @Override
    public void initialize() {
        robotContainer.driveSubsystem.setDriveEnabled(false);

        double reversed = isReversed ? -1 : 1;
        robotContainer.driveSubsystem.drive(
            DriveConstants.AUTO_LEVEL_DEFAULT_SPEED * reversed,
            0,
            0,
            true,
            false,
            true
        );
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        samples.addSample(getAdjustedAngle());
        Float rate = samples.getAverageRate();

        System.out.println("AUTOLEVEL RATE: " + samples.getAverageRate() + " | THRESHOLD: " + DriveConstants.AUTO_LEVEL_END_RATE);
        ArrayList<Float> samp = samples.getSamples();
        for (int i = 0; i < samp.size(); i++) {
            System.out.print(samp.get(i) + ",");
        }
        System.out.print("\n");
        return rate >= DriveConstants.AUTO_LEVEL_END_RATE;

        // TODO: this should solve the one-sided ramp issue, if it ends up being an issue
        /*if (isReversed) {
            if (rate >= DriveConstants.AUTO_LEVEL_END_RATE) {
                cancel();
            }
        } else {
            if (rate <= DriveConstants.AUTO_LEVEL_END_RATE) {
                cancel();
            }
        }*/
    }

    // converts degrees/s    ->    degrees/20ms (20ms = 1 tick)
    public static float toDegreesPerTick(float degreesPerSecond) {
        return degreesPerSecond / (1000 / 20);
    }

    private float getAdjustedAngle() {
        return robotContainer.driveSubsystem.ahrs.getPitch() - DriveConstants.GYRO_PITCH;
    }

    @Override
    public void end(boolean interrupted) {
        robotContainer.driveSubsystem.setX();
    }
}
