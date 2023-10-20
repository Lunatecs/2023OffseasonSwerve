// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalanceCommand extends PIDCommand {
  /** Creates a new AutoBalanceCommand. */
  private SwerveDrive swerve;

  public AutoBalanceCommand(SwerveDrive swerve) {
    super(
        // The controller that the command will use
        new PIDController(0.195, 0.0165, 0),
        // This should return the measurement
        () -> swerve.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> swerve.getZeroAngle(),
        // This uses the output
        output -> {

          SmartDashboard.putNumber("Auto output", output);
          if(Math.abs(output)>0.325) {
              output = Math.signum(output) * 0.4;
          }
          if(swerve.getPitch() == swerve.getZeroAngle()) {
            swerve.stop();
          }
          swerve.drive(new Translation2d(output, 0), 0, true, false);
        }
      );
    addRequirements(swerve);
    this.getController().setTolerance(1);
    this.swerve = swerve;
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("balance error", this.getController().getPositionError());
    SmartDashboard.putNumber("balance set point", this.getController().getSetpoint());
    SmartDashboard.putNumber("balance  pitch", swerve.getPitch());
    SmartDashboard.putBoolean("balance isfinished", Math.abs(this.getController().getPositionError()) <= this.getController().getPositionTolerance());
    super.execute();
  }


  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    swerve.stop();
    System.out.println("AutoBalance END");
  }
}
