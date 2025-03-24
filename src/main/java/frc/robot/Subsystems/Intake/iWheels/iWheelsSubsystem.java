package frc.robot.Subsystems.Intake.iWheels;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;

public class iWheelsSubsystem extends SubsystemBase {
    private static iWheelsSubsystem instance;
    private SparkMax motorIW;

    // Private constructor for singleton pattern
    private iWheelsSubsystem() {
        motorIW = new SparkMax(RobotMap.INTAKE_WHEELS_MOTOR_ID, MotorType.kBrushless);
    }

    // Singleton instance getter
    public static iWheelsSubsystem getInstance() {
        if (instance == null) {
            instance = new iWheelsSubsystem();
        }
        return instance;
    }

    // Method to set motor speed
    public void setSpeed(double speed) {
        motorIW.set(speed);
    }

    // Method to stop the motor
    public void stopMotor() {
        motorIW.set(0);
    }

    // Method to reverse the motor direction
    public void reverseMotor() {
        motorIW.set(-motorIW.get());
    }

    // Method to run the motor at a specific speed for a duration
    public void runForDuration(double speed, double durationSeconds) {
        setSpeed(speed);
        Timer.delay(durationSeconds);
        stopMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}