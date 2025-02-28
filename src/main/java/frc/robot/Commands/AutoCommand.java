package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystems;
import frc.robot.Subsystems.Drivetrain.DrivetrainSubsystem;

public class AutoCommand extends Command{
    DrivetrainSubsystems drive = new DrivetrainSubsystems();
    private Timer timer;
    private double seconds = 1.0;
    
    public AutoCommand() {
        timer = new Timer();
    }
    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        DrivetrainSubsystem.getInstance().setArcade(0.5, 0.0);
    }

    @Override
    public void end(boolean isInterrupted ) {
        DrivetrainSubsystem.getInstance().setArcade(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= seconds;
    }
}
