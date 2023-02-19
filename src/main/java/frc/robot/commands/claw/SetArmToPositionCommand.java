package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// TODO: only works if the desired position is ABOVE the current position*****
public class SetArmToPositionCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double desiredPosition;
    private boolean hold;

    /**
     * @param desiredPosition a relative value from 0-1
     * @param hold Whether or not to hold the arm in place once the position is reached
     */
    public SetArmToPositionCommand(ArmSubsystem armSubsystem, double desiredPosition, boolean hold) {
        this.armSubsystem = armSubsystem;
        this.desiredPosition = desiredPosition;
        this.hold = hold;
    }

    @Override
    public void execute() {
        double relativePos = armSubsystem.getRelativePosition();
        double armSpeed = (relativePos > desiredPosition - ArmConstants.FINE_SPEED_MARGIN &&
            relativePos < desiredPosition + ArmConstants.FINE_SPEED_MARGIN) ? ArmConstants.FINE_SPEED : ArmConstants.COARSE_SPEED;
        boolean desiresUpward = relativePos < desiredPosition;

        armSubsystem.setMotors((desiresUpward ? -1 : 1) * armSpeed);
    }

    @Override
    public boolean isFinished() {
        if (hold) {
            return false;
        }

        double relativePos = armSubsystem.getRelativePosition();
        return relativePos > (desiredPosition - ArmConstants.RELATIVE_ACCEPTABLE_MARGIN) &&
            relativePos < (desiredPosition + ArmConstants.RELATIVE_ACCEPTABLE_MARGIN);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMotors(0);
    }
}
