// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.fasterxml.jackson.databind.deser.impl.PropertyValue;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunElevator extends Command {
  /** Creates a new RunElevator. */
  private final Joystick m_controller;
  private final ElevatorSubsystem m_elevator;
  public RunElevator(ElevatorSubsystem elevator, Joystick controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_controller = controller;
    m_elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.getRawButton(3)) {
      m_elevator.runElevator(0.25);
    }
    else if(m_controller.getRawButton(4)){
      m_elevator.runElevator(-0.25);
    }
    else {
      m_elevator.runElevator(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
