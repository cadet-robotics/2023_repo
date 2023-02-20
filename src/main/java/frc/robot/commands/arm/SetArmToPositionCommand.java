package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;

// TODO: only works if the desired position is ABOVE the current position*****
public class SetArmToPositionCommand extends CommandBase {
    public static SetArmToPositionCommand global;

    private final ArmSubsystem armSubsystem;
    private final ClawSubsystem clawSubsystem;

    public double desiredPosition;
    private final boolean hold;

    /**
     * @param desiredPosition a relative value from 0-1
     * @param hold Whether or not to hold the arm in place once the position is reached
     */
    public SetArmToPositionCommand(ClawSubsystem clawSubsystem, ArmSubsystem armSubsystem, double desiredPosition, boolean hold) {
        this.clawSubsystem = clawSubsystem;
        this.armSubsystem = armSubsystem;
        this.desiredPosition = desiredPosition;
        this.hold = hold;

        addRequirements(clawSubsystem, armSubsystem);

        if (global != null) {
            global.cancel();
        }
        global = this;
    }

    @Override
    public void initialize() {
        if (desiredPosition < ArmConstants.INTAKE_RETRACT_POSITION) {
            clawSubsystem.setClawShut(true);
        }
    }

    @Override
    public void execute() {
        double relativePos = armSubsystem.getRelativePosition();
        boolean desiresUpward = relativePos < desiredPosition;
        boolean fineSpeed = (relativePos > desiredPosition - ArmConstants.FINE_SPEED_MARGIN &&
            relativePos < desiredPosition + ArmConstants.FINE_SPEED_MARGIN);
        
        double armSpeed;
        if (desiresUpward) {
            armSpeed = fineSpeed ? ArmConstants.FINE_SPEED_UP : ArmConstants.COARSE_SPEED_UP;
            armSpeed *= -1;
        } else {
            armSpeed = fineSpeed ? ArmConstants.FINE_SPEED_DOWN : ArmConstants.COARSE_SPEED_DOWN;
            armSpeed += relativePos < ArmConstants.FULL_DOWN_POSITION ? ArmConstants.REVERSE_SPEED_ADDITION : 0;
        }

        /*SmartDashboard.putNumber("arm/desiredPosition", desiredPosition);
        SmartDashboard.putNumber("arm/relativePos", relativePos);
        SmartDashboard.putBoolean("arm/desiresUpward", desiresUpward);
        SmartDashboard.putBoolean("arm/fineSpeed", fineSpeed);
        SmartDashboard.putNumber("arm/armSpeed", armSpeed);*/
        
        armSubsystem.setMotors(armSpeed);
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
        global = null;
    }
}
