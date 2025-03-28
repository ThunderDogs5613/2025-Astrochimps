package frc.robot.Subsystems.Intake.iArm.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake.iArm.iArmSubsystem;

public class PrintState extends Command {

  public static double position;

  public PrintState() {
    addRequirements(iArmSubsystem.getInstance());
  }


  @Override
  public void initialize() {
    //ArmSubsystem.getInstance().setPower(0);
    System.out.println("initialize print Statementy");
  }


  @Override
  public void execute() {
    position = iArmSubsystem.getInstance().getIntakeArmPos();
    System.out.println(position);
    System.out.println("Arm Position"); }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
