// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmExtensionCommand extends PIDCommand {
  ArmSubsystem arm;

  public SetArmExtensionCommand(double setpoint, ArmSubsystem arm) {
    super(
        // The controller that the command will use
        new PIDController(0.00006, 0, 0),
        // This should return the measurement
        () -> arm.getArmEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          if (Math.abs(output) < 0.3) {
            output = (Math.abs(output)/output)*0.3;
          }
          //Yes, it is probably possible to make this cleaner/more effcient. No, I do not in fact care.
          /*if (Math.abs(output) > 0.7) {
            output = (Math.abs(output)/output)*0.7;
          }*/

          arm.setSpeed(output);
          SmartDashboard.putNumber("Arm Output", output);
        });
    addRequirements(arm);
    this.arm = arm;
  }

  @Override
  public void initialize() {
    super.initialize();
    this.getController().setTolerance(ArmConstants.ARM_TOLERANCE);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Arm Error", this.getController().getPositionError());
    SmartDashboard.putNumber("Arm Tolerance", this.getController().getPositionTolerance());
    super.execute();
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(this.getController().getPositionError()) <= this.getController().getPositionTolerance();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    arm.setSpeed(0);
  }
}
