// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utils.SetPointSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetElevatorPositionCommand extends PIDCommand {

  ElevatorSubsystem elevator;

  public SetElevatorPositionCommand(ElevatorSubsystem elevator, double setpoint, double p, double i, double d) {
    super(
        // The controller that the command will use
        new PIDController(p, i, d),
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
          SmartDashboard.putNumber("Elevator Output", output);
        });
        this.elevator = elevator;
        addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` her6e.
  }

  @Override
  public void initialize() {
    super.initialize();
    this.getController().setTolerance(500);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Elevator Error", this.getController().getPositionError());
    SmartDashboard.putNumber("Elevator Tolerance", this.getController().getPositionTolerance());
    if(elevator.tripLimit() && this.getController().calculate(elevator.getElevatorEncoder()) < 0) {
      elevator.setSpeed(0);
      return;
    }
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    elevator.setSpeed(0);
  }
}
