// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public final class Constants {}  

    public final class DrivetrainConstants{
        public static final double RobotMaxSpeed = 14;
    }
    public final class RobotConstants{
        public static final double kRobotMass = 50.0;
        public static final double kRobotWidth = 2.0;
        public static final double kRobotLength = 3.0; 
        public static final double kRobotDrag = 0.1;
        public static final double kRobotMomentOfInertia = 11.0;
        public static final double kRobotMaxSpeed = 10.0;
        public static final double kRobotMaxAcceleration = 11.0;
    }

    public final class ElevatorConstants{
            public static final double kElevatorKp = 5;//5
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0;//
    public static final double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static final double kElevatorkS = 0.02;
    public static final double kElevatorkG = 0.9;
    public static final double kElevatorkV = 3.8;
    public static final double kElevatorkA = 0.17;
    public static final double kElevatorRampRate = 0.1;
    public static final double kElevatorGearing = 60.0;
    public static final double kElevatorCarriageMass = 7.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = 10.25;
    public static final double kElevatorLength = Inches.of(33).in(Meters);
    public static final Distance kElevatorStartingHeightSim = Meters.of(0.0);
    public static final Angle kElevatorStartingAngle = Degrees.of(-90);
    public static final Distance kLaserCANOffset          = Inches.of(3);
    public static final double kElevatorDefaultTolerance = Inches.of(1).in(Meters);

    public static double kLowerToScoreHeight =  Units.inchesToMeters(6);;

    }

    public final class ArmConstants{
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double l1 = 0;
        public static final double l2 = 0;
        public static final double l3 = 0;
        public static final double l4 = 0;
        public static final double stow = 0;

        public enum ArmPos {
            L1, L2, L3, L4, STOW, HOLD;

            public double getPosition() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
            }
        }
    }

    public final class IntakeConstants {
        public static final double kP = 0;//5
        public static final double kI = 0;
        public static final double kD = 0;//
    public static double kLowerToScoreHeight =  Units.inchesToMeters(6);
    // Additional constants
    public static final double kIntakeMaxSpeed = 0.25; // Maximum speed of the intake
    public static final double kIntakeMinSpeed = 0.1; // Minimum speed of the intake

    // Method to calculate intake speed based on some input
    public static double calculateIntakeSpeed(double input) {
        // Example PID control calculation (simplified)
        double error = input - kLowerToScoreHeight;
        double speed = kIntakeKp  + kIntakeKi  () + kIntakeKd  ( / 2);

        // Clamp the speed to the min and max values
        if (speed > kIntakeMaxSpeed) {
            speed = kIntakeMaxSpeed;
        } else if (speed < kIntakeMinSpeed) {
            speed = kIntakeMinSpeed;
        }


        public static final double speed = 0.25;
    }

    public static double kLowerToScoreHeight = Units.inchesToMeters(6);

    // Additional constants 
    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAcceleration = 3.0;
    public static final double kMaxVoltage = 10.0;
    public static final double kMaxAngularSpeed = 3.0;
    public static final double kMaxAngularAcceleration = 3.0;
    public static final double kMaxAngularVoltage = 10.0;
    public static final double kMaxPosition = 3.0;
    public static final double kMaxVelocity = 3.0;
    public static final double kMaxCurrent = 3.0;
    public static final double kMaxForce = 3.0;
    public static final double kMaxPressure = 3.0;
    public static final double kMaxFlow = 3.0;
    public static final double kMaxTemperature = 3.0;
    public static final double kMaxPower = 3.0;
    public static final double kMaxEnergy = 3.0;
    public static final double kMaxTorque = 3.0;
    public static final double kMaxAngularCurrent = 3.0;
    public static final double kMaxAngularPower = 3.0;
    public static final double kMaxAngularEnergy = 3.0;
    public static final double kMaxAngularTorque = 3.0;
    public static final double kMaxAngularMomentum = 3.0;
    public static final double kMaxAngularImpulse = 3.0;
    public static final double kMaxAngularInertia = 3.0;
    public static final double kMaxAngularStiffness = 3.0;
    public static final double kMaxAngularDamping = 3.0;
    public static final double kMaxAngularCompliance = 3.0;
    public static final double kMaxAngularVelocity = 3.0;



    // Method to set the speed
    public static void setSpeed(double speed) {
        // Code to set the speed of the intake/outtake mechanism
        // This will depend on your specific robot control code
    }

    // Method to read the throttle value
    public static double getThrottleValue() {
        // Code to read the throttle value
        // This will depend on your specific robot control code
        return 0.0; // Placeholder value
    }

    // Method to control the arm using the throttle
    public static void controlArmWithThrottle() {
        double throttleValue = getThrottleValue();
        setSpeed(throttleValue);
    }
}
