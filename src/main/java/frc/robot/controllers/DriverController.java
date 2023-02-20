package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class DriverController extends BaseController {
    public DriverController(int port, RobotContainer robotContainer) {
        super(port, robotContainer);
    }

    public void initBindings() {
        // THIS IS ALL TEMPORARY; this will all (hopefully) be replaced
        
        // arm run thing
        button(5).whileTrue(new RunArmMotorTemp(robotContainer, false));
        button(6).whileTrue(new RunArmMotorTemp(robotContainer, true));

        button(7).onTrue(Commands.runOnce(() -> {
            robotContainer.clawSubsystem.setClawShut(true);
        }));

        button(8).onTrue(Commands.runOnce(() -> {
            robotContainer.clawSubsystem.setClawShut(false);
        }));


    }
}

class RunArmMotorTemp extends CommandBase {
    private RobotContainer robotContainer;
    private boolean inverse;

    public RunArmMotorTemp(RobotContainer robotContainer, boolean inverse) {
        this.robotContainer = robotContainer;
        this.inverse = inverse;
    }

    @Override
    public void initialize() {
        robotContainer.armSubsystem.getMainMotor().set(0.4 * (inverse ? -1 : 1));
        robotContainer.armSubsystem.getHelperMotor().set(0.4 * (inverse ? -1 : 1));
    }

    @Override
    public void end(boolean interrupted) {
        robotContainer.armSubsystem.getMainMotor().set(0);
        robotContainer.armSubsystem.getHelperMotor().set(0);
    }
}