// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorBottomCommand extends CommandBase {
  double setpoint;
  ElevatorSubsystem elevator;
  boolean isFinished = false;
  public ElevatorBottomCommand(double setpoint, ElevatorSubsystem elevator) {
    this.setpoint = setpoint;
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(setpoint == ElevatorConstants.BOTTOM && elevator.getElevatorEncoder() < 0) {
      elevator.setSpeed(0.15);
    }else {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
