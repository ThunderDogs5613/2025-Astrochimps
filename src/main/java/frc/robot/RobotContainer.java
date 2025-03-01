// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AutoCommand;
import frc.robot.Constants.*;
import frc.robot.Subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.Subsystems.Drivetrain.States.OpenLoopState;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Constants.IntakeConstants;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  DrivetrainSubsystem drive;

  ElevatorSubsystem elevSystem;

  CommandGenericHID driveStick = ControllerMap.getDriveStick();

  public RobotContainer() {
    initializeSubsystems();
    configureBindings();
    setDefaultCommands();
    autoChooser.setDefaultOption("Autonomous", new AutoCommand());
  }

  private void initializeSubsystems() {
    drive = DrivetrainSubsystem.getInstance();
    elevSystem = new ElevatorSubsystem();
  }

  private void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(drive, new OpenLoopState());
  }

  private void configureBindings() {
    Trigger test = driveStick.button(ControllerMap.DriveController.Button.B3);
    test.whileTrue(elevSystem.runElevator(0.25));

    Trigger test2 = driveStick.button(ControllerMap.DriveController.Button.B4);
    test2.whileTrue(elevSystem.runElevator(-0.25));

    Trigger test3 = driveStick.button(ControllerMap.DriveController.Button.B5);
    test3.whileTrue(elevSystem.runElevator(0));

    Trigger buttonB6 = driveStick.button(ControllerMap.DriveController.Button.B6);
    Trigger buttonB7 = driveStick.button(ControllerMap.DriveController.Button.B7);
    Trigger buttonB8 = driveStick.button(ControllerMap.DriveController.Button.B8);

    buttonB6.whileTrue(() -> IntakeConstants.controlIntake(true, false, false, getCurrentHeight()));
    buttonB7.whileTrue(() -> IntakeConstants.controlIntake(false, true, false, getCurrentHeight()));
    buttonB8.whileTrue(() -> IntakeConstants.controlIntake(false, false, true, getCurrentHeight()));
  }

  private double getCurrentHeight() {
    // Placeholder method to get the current height of the intake
    // Replace with actual implementation
    return 0.0;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

