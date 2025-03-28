package frc.robot.Subsystems.Intake.iArm.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.Intake.iArm.iArmSubsystem;
import frc.robot.Constants.Constants.IntakeConstants;
public class PositionState extends Command{

            private IntakeConstants.ArmPos position;
            private double holdPosition;
        
        
          
            public PositionState(IntakeConstants.ArmPos position) {
              addRequirements(iArmSubsystem.getInstance());  
          
              this.position = position;
            }
          
            @Override
            public void initialize() {
          
              holdPosition = iArmSubsystem.getInstance().getIntakeArmPos();
        
              iArmSubsystem.enable();
        
              switch(position) {
                case L1 :
              iArmSubsystem.getInstance().setArmSetpoint(Constants.ArmConstants.l1);
              //iArmSubsystem.getInstance().setFeedForward(0.02);
              break;
      
            case L2 :
          iArmSubsystem.getInstance().setArmSetpoint(Constants.ArmConstants.l2);
        //  iArmSubsystem.getInstance().setFeedForward(0.01);
          break;  
          
        case L3 :
          iArmSubsystem.getInstance().setArmSetpoint(Constants.ArmConstants.l3);
      //    iArmSubsystem.getInstance().setFeedForward(0.2);
          break;

        case STOW :
          iArmSubsystem.getInstance().setArmSetpoint(Constants.ArmConstants.stow);
        //  iArmSubsystem.getInstance().setFeedForward(0.2);
          break;

        case HOLD :
        holdPosition = iArmSubsystem.getInstance().getIntakeArmPos();
        iArmSubsystem.getInstance().setArmSetpoint(holdPosition);
       // iArmSubsystem.getInstance().setFeedForward(0.0);
          break;
      }
      iArmSubsystem.getInstance();
      iArmSubsystem.enable();
    }
  
    public void execute() {
    }
  
    @Override
    public void end(boolean interrupted) {
      iArmSubsystem.getInstance().disable();
      iArmSubsystem.getInstance().setPower(0);
    }
}