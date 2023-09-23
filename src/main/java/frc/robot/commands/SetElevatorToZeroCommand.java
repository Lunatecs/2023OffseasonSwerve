// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorToZeroCommand extends PIDCommand {
  /** Creates a new SetElevatorToZeroCommand. */
  ElevatorSubsystem elevator;
  boolean isFinished = false;
  public SetElevatorToZeroCommand(ElevatorSubsystem elevator) {
    super(
        // The controller that the command will use
        new PIDController(0.00005, 0, 0),
        // This should return the measurement
        () ->  elevator.getElevatorEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> ElevatorConstants.BOTTOM,
        // This uses the output
        output -> {
          if(elevator.getElevatorEncoder() > -10) {
            output = 0;
          }else if(elevator.getElevatorEncoder() >= ElevatorConstants.BOTTOM) {
            output = 0.15;
          }
          SmartDashboard.putNumber("Bottom Elevstor Output", output);
          elevator.setSpeed(output);
        });
    addRequirements(elevator);
    this.elevator = elevator;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getElevatorEncoder() > -10;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    elevator.setSpeed(0);
  }
}
