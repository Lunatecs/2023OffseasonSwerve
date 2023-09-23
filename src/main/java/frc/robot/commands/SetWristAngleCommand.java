// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetWristAngleCommand extends PIDCommand {
  /** Creates a new SetWristAngleCommand. */

  WristSubsystem wrist;

  public SetWristAngleCommand(WristSubsystem wrist, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(0.0001, 0, 0),
        // This should return the measurement
        () -> wrist.getWristEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          if (Math.abs(output) < 0.1) {
            output = (Math.abs(output)/output) * 0.1;
          }
          wrist.turnWrist(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
    this.getController().setTolerance(1000);
    this.wrist = wrist;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("Wrist Command Error", this.getController().getPositionError());
    SmartDashboard.putNumber("Wrist Command Encoder", this.wrist.getWristEncoder());
    SmartDashboard.putNumber("Wrist Command SetPoint", this.getController().getSetpoint());
    return Math.abs(this.getController().getPositionError()) <= this.getController().getPositionTolerance();
  }
}
