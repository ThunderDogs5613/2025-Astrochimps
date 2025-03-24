package frc.robot.Subsystems.Drivetrain.States;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ControllerMap;
import frc.robot.Subsystems.Drivetrain.DrivetrainSubsystem;

public class OpenLoopState extends Command {
        
    DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance();
    
    public OpenLoopState() {
         addRequirements(drive);
    }

    @Override
    public void execute() {
        double throttle = ControllerMap.getDriveStick().getRawAxis(ControllerMap.DriveController.Axis.STICK_Y)*-0.8;
        double rotation = ControllerMap.getDriveStick().getRawAxis(ControllerMap.DriveController.Axis.STICK_X)*-0.75;
            
        DrivetrainSubsystem.getInstance().setArcade(throttle, rotation);
    }
}