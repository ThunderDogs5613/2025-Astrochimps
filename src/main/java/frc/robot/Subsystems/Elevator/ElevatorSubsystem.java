package frc.robot.Subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//7121 Help
//SparkMax m_elevetor = new SparkMax();

public class ElevatorSubsystem extends SubsystemBase {
    // setUp
    private final SparkMax m_motor = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax f_motor = new SparkMax(6, SparkLowLevel.MotorType.kBrushless);

    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final PIDController m_controller = new PIDController(
            frc.robot.Constants.Constants.ElevatorConstants.kElevatorKp,
            frc.robot.Constants.Constants.ElevatorConstants.kElevatorKi,
            frc.robot.Constants.Constants.ElevatorConstants.kElevatorKd);

    public ElevatorSubsystem() {

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20)
                .openLoopRampRate(frc.robot.Constants.Constants.ElevatorConstants.kElevatorRampRate);

        SparkMaxConfig f_config = new SparkMaxConfig();
        f_config.smartCurrentLimit(20)
                .openLoopRampRate(frc.robot.Constants.Constants.ElevatorConstants.kElevatorRampRate)
                .follow(5, true);

        m_motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);//
        f_motor.configure(f_config, SparkBase.ResetMode.kNoResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    }

    public double getPositionMeters() {
        return m_encoder.getPosition()
                * (2 * Math.PI * frc.robot.Constants.Constants.ElevatorConstants.kElevatorDrumRadius)
                / frc.robot.Constants.Constants.ElevatorConstants.kElevatorGearing;
    }

    public double getVelocityMetersPerSecond() {
        return (m_encoder.getVelocity() / 60)
                * (2 * Math.PI * frc.robot.Constants.Constants.ElevatorConstants.kElevatorDrumRadius)
                / frc.robot.Constants.Constants.ElevatorConstants.kElevatorGearing;
    }

    // public void reachGoal(double goal) {
    //     m_motor.set(m_controller.calculate(getPositionMeters()));
    // }

    // public Command setGoal(double goal) {
    //     return run(() -> reachGoal(goal));
    // }

    // public Command setElevatorHeight(double height) {
    //     return setGoal(height).until(() -> aroundHeight(height));
    // }

    // public boolean aroundHeight(double height) {
    //     return aroundHeight(height, frc.robot.Constants.Constants.ElevatorConstants.kElevatorDefaultTolerance);
    // }

    // public boolean aroundHeight(double height, double tolerance) {
    //     return MathUtil.isNear(height, getPositionMeters(), tolerance);
    // }

    // public Command setHeight(double position) {
    // return run(
    // () -> {
    // m_CLController.setReference(position, ControlType.kPosition);
    // }
    // );
    // }

    /**
     * Stop the control loop and motor output.
     */
    public void stop() {
        m_motor.set(0.0);
    }

    public void runElevator(double output) {
        m_motor.set(output);
    }

    /**
     * Update telemetry, including the mechanism visualization.
     */
    public void updateTelemetry() {}

    @Override
    public void periodic() {
    }
}
