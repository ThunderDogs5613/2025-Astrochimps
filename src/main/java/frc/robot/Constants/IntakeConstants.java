package frc.robot.Constants;

import edu.wpi.first.math.util.Units;

public final class IntakeConstants {
    public static final double kIntakeKp = 0; // 5
    public static final double kIntakeKi = 0;
    public static final double kIntakeKd = 0; // 

    public static final double kLowerToScoreHeight = Units.inchesToMeters(6);

    // Additional constants
    public static final double kIntakeMaxSpeed = 0.5; // Maximum speed of the intake
    public static final double kIntakeMinSpeed = 0.1; // Minimum speed of the intake

    // Method to calculate intake speed based on some input
    public static double calculateIntakeSpeed(double currentHeight) {
        // Calculate the error as the difference between the desired height and the current height
        double error = kLowerToScoreHeight - currentHeight;
        
        // Example PID control calculation (simplified)
        double speed = kIntakeKp * error + kIntakeKi * (error * error) + kIntakeKd * (error / 2);

        // Clamp the speed to the min and max values
        if (speed > kIntakeMaxSpeed) {
            speed = kIntakeMaxSpeed;
        } else if (speed < kIntakeMinSpeed) {
            speed = kIntakeMinSpeed;
        }

        return speed;
    }

    // Method to control the intake using buttons B6, B7, and B8
    public static void controlIntake(boolean buttonB6, boolean buttonB7, boolean buttonB8, double currentHeight) {
        double intakeSpeed = 0;

        if (buttonB6) {
            // Button B6 pressed: Increase intake speed
            intakeSpeed = calculateIntakeSpeed(currentHeight) + 0.1;
        } else if (buttonB7) {
            // Button B7 pressed: Decrease intake speed
            intakeSpeed = calculateIntakeSpeed(currentHeight) - 0.1;
        } else if (buttonB8) {
            // Button B8 pressed: Stop intake
            intakeSpeed = 0;
        }

        // Set the intake speed (assuming a method setIntakeSpeed exists)
        setIntakeSpeed(intakeSpeed);
    }

    // Placeholder method to set the intake speed
    private static void setIntakeSpeed(double speed) {
        // Implementation to set the intake speed
        System.out.println("Setting intake speed to: " + speed);
    }
}
