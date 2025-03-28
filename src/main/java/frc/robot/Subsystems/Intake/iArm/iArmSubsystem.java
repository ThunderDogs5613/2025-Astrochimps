package frc.robot.Subsystems.Intake.iArm;

import com.revrobotics.spark.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.Constants;
import frc.robot.Constants.RobotMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class iArmSubsystem extends SubsystemBase {
    private static boolean isUsingPID = false;
    private SparkMax motorIA;
    private DutyCycleEncoder encoder;
    //private RelativeEncoder encoder;
    private PIDController armPID = new PIDController(Constants.IntakeConstants.kP, Constants.IntakeConstants.kI, Constants.IntakeConstants.kD);

    private static iArmSubsystem instance;

    private iArmSubsystem() {
    
//
      motorIA = new SparkMax(RobotMap.INTAKE_ARM_MOTOR_ID, MotorType.kBrushless);
      encoder = new DutyCycleEncoder(RobotMap.INTAKE_ARM_ENCODER_ID); 
      //encoder = motorIA.getEncoder();
    }

    public static synchronized iArmSubsystem getInstance() {
        if(instance == null){
          instance = new iArmSubsystem();
        }
        return instance;
    }
    
    public void setPower(double power) {
      motorIA.set(power);
    }

    protected double getMeasurement() {
      return getIntakeArmPos();
    } 

    public double getIntakeArmPos() {
      return encoder.get();
      //return encoder.getPosition();
    }
    
    public static void enable() {
      isUsingPID = true;
    }

    public void disable() {
      isUsingPID = false;
    }

    public void setArmSetpoint(double setpoint) {
      armPID.setSetpoint(setpoint);
    }

    public double getArmSetpoint() {
      return armPID.getSetpoint();
    }
    
    public void periodic() {
      if(isUsingPID) {
        setPower(armPID.calculate(getMeasurement()));
      }
    }

    public void setFeedForward(double d) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'setFeedForward'");
    }
}
