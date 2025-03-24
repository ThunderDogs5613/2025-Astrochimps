// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AutoCommand;
import frc.robot.Constants.*;
import frc.robot.Constants.Constants.ArmConstants.ArmPos;
import frc.robot.Subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.Subsystems.Drivetrain.States.OpenLoopState;
import frc.robot.Subsystems.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Intake.iArm.iArmSubsystem;
import frc.robot.Subsystems.Intake.iArm.States.PositionState;
import frc.robot.Subsystems.Intake.iArm.States.PrintState;
import frc.robot.Subsystems.Intake.iWheels.iWheelsSubsystem;
import frc.robot.Subsystems.Intake.iWheels.States.InState;
import frc.robot.Subsystems.Intake.iWheels.States.OutState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.BooleanEvent;


public class RobotContainer {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  DrivetrainSubsystem drive;
  iArmSubsystem iArm;
  iWheelsSubsystem iwheels;
  ElevatorSubsystem elevator;

  CommandGenericHID driveController = ControllerMap.getDriveStick();
  private final Joystick driveStick = new Joystick(0); // Replace 0 with the correct port
  private final iArmSubsystem iarm = iArmSubsystem.getInstance();

  public RobotContainer() {
    initializeSubsystems();
    configureBindings();
    setDefaultCommands();
  }

  private void initializeSubsystems() {
    drive = DrivetrainSubsystem.getInstance();
    iArm = iArmSubsystem.getInstance();
    iwheels = iWheelsSubsystem.getInstance();
    elevator = new ElevatorSubsystem();
    }

  private void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(drive, new OpenLoopState());
    CommandScheduler.getInstance().setDefaultCommand(iarm, new PrintState());
  }

  
  private void configureBindings(){
    Trigger test = driveController.button(ControllerMap.DriveController.Button.B3);

    test.whileTrue(
      elevator.runElevator(0.25)
    );

    Trigger test2 = new Trigger(() -> driveStick.getRawButton(ControllerMap.DriveController.Button.B4));

    test2.whileTrue(
      elevator.runElevator(-0.25)
    );

    Trigger test3 = driveController.button(ControllerMap.DriveController.Button.B5);

    test3.whileTrue(
      elevator.runElevator(0.25)
    );

    Trigger outtake = driveController.button(ControllerMap.DriveController.Button.TRIGGER).whileTrue(
      new OutState().repeatedly()
    );
  
    Trigger intake = driveController.button(ControllerMap.DriveController.Button.B6).whileTrue(
      new InState().repeatedly()
    );
    //test3.whileFalse(
    //  Commands.runOnce(() -> iarm.setIArmPosition(ArmPos.STOW))
    //);



  /*  Trigger test4 = new Trigger(() -> driveStick.getRawButton(ControllerMap.DriveController.Button.B6));
    test4.whileTrue(
      Commands.run(() -> iarm.setSubsystem(0.25))
    );

    Trigger test5 = new Trigger(() -> driveStick.getRawButton(ControllerMap.DriveController.Button.B7));
    test5.whileTrue(
      Commands.run(() -> iarm.setSubsystem(-0.25))
    );*/
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

