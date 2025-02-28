
package frc.robot.Constants;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Constants.ElevatorConstants;

public class RobotMath
{

  public static class Elevator
  {

    /**
     * Convert {@link Distance} into {@link Angle}
     *
     * @param distance Distance, usually Meters.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertDistanceToRotations(Distance distance)
    {
      // m/(2*pi*r)*g = e
      return Rotations.of((distance.in(Meters) /
                          (frc.robot.Constants.Constants.ElevatorConstants.kElevatorDrumRadius * 2.0 * Math.PI)) *
                          frc.robot.Constants.Constants.ElevatorConstants.kElevatorGearing);
    }

    /**
     * Convert {@link Angle} into {@link Distance}
     *
     * @param rotations Rotations of the motor
     * @return {@link Distance} of the elevator.
     */
    public static Distance convertRotationsToDistance(Angle rotations)
    {
      return Meters.of((rotations.in(Rotations) / frc.robot.Constants.Constants.ElevatorConstants.kElevatorGearing) *
                       (frc.robot.Constants.Constants.ElevatorConstants.kElevatorDrumRadius * 2 * Math.PI));
    }
  }
}
