// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetEndableElevatorPositionCommand extends PIDCommand {
  /** Creates a new SetEndableElevatorPositionCommand. */
  ElevatorSubsystem elevator;
  public SetEndableElevatorPositionCommand(ElevatorSubsystem elevator, double setpoint, double p) {
    super(
        // The controller that the command will use
        new PIDController(p, 0, 0),
        // This should return the measurement
        () -> elevator.getElevatorEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          if (Math.abs(output) < 0.1) {
            output = (Math.abs(output)/output) * 0.1;
          }
          elevator.setSpeed(output);
        });
        addRequirements(elevator);
        this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    super.initialize();
    this.getController().setTolerance(500);
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Ended Elevator Command", Math.abs(this.getController().getPositionError()) <= this.getController().getPositionTolerance());
    SmartDashboard.putNumber("Endable Elevator Error", this.getController().getPositionError());
    SmartDashboard.putNumber("Endable Elevator Tolerance", this.getController().getPositionTolerance());
    return Math.abs(this.getController().getPositionError()) <= this.getController().getPositionTolerance();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    elevator.setSpeed(0);
  }
}
