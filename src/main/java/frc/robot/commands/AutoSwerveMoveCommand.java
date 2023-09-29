// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class AutoSwerveMoveCommand extends CommandBase {
  /** Creates a new AutoSwerveMoveCommand. */
  private PIDController drivePID;
  private SwerveDrive drive;
  private double distance;
  private double kP;


  public AutoSwerveMoveCommand(SwerveDrive drive, double distance, double kP) {
    this.drive = drive;
    this.distance = distance;
    this.kP = kP;
    addRequirements(drive);
    drivePID = new PIDController(kP, 0, 0);
    drivePID.setSetpoint(distance * ((2048*6.75)/(4*Math.PI)));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setYaw(180.0);
    drive.resetModulesToAbsolute();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = Math.abs(drive.getFLDriveEncoder());
    drive.drive(
      new Translation2d(drivePID.calculate(error), 0).times(Constants.Swerve.maxSpeed),
      0, 
      true, 
      false
    );
    SmartDashboard.putNumber("Auto Error", drivePID.getPositionError());
    SmartDashboard.putNumber("Auto encoder value", error);
    SmartDashboard.putNumber("SetPoint", drivePID.getSetpoint());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
