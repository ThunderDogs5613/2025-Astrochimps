package frc.robot.Subsystems.Intake.iWheels.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.Intake.iWheels.iWheelsSubsystem;
import frc.robot.RobotContainer;
import frc.robot.Robot;


public class IdleState extends Command {
    
    private iWheelsSubsystem intake = iWheelsSubsystem.getInstance();

    public IdleState() {
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(0);
    }

}