package frc.robot.controllers;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.RobotContainer;

public class BaseController extends CommandPS4Controller {
    protected RobotContainer robotContainer;

    public BaseController(int port, RobotContainer robotContainer) {
        super(port);

        this.robotContainer = robotContainer;
    }
}
