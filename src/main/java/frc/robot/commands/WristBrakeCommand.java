// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.SetPointSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristBrakeCommand extends PIDCommand {
  /** Creates a new WristBrakeCommand. */
  SetPointSupplier setpoint;
  WristSubsystem wrist;

  public WristBrakeCommand(SetPointSupplier setpoint, WristSubsystem wrist) {
    super(
        // The controller that the command will use
        new PIDController(0.0001, 0, 0),
        // This should return the measurement
        () -> wrist.getWristEncoder(),
        // This should return the setpoint (can also be a constant)
          setpoint,
        // This uses the output
        output -> {
          // Use the output here
          if(wrist.tripLimit() && output < 0) {
            wrist.turnWrist(0);
          } else {
            wrist.turnWrist(output);
          }
          
        });

        this.wrist = wrist;
        this.setpoint = setpoint;
        addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    setpoint.setSetPoint(wrist.getWristEncoder());
    wrist.turnWrist(0);
    super.initialize();
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("wrist error", this.getController().getPositionError());
    SmartDashboard.putNumber("Setpoint", setpoint.getAsDouble());
    SmartDashboard.putBoolean("Trippin Bawls", wrist.tripLimit());
    if (wrist.tripLimit()) {
      setpoint.setSetPoint(setpoint.getAsDouble()-200);
    }
    super.execute();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
