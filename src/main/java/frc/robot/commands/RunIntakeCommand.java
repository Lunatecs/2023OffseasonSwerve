// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Date;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class RunIntakeCommand extends CommandBase {
  /** Creates a new RunIntakeCommand. */
  IntakeSubsystem intake;
  LEDSubsystem led;
  DoubleSupplier intakeSpeed;
  boolean finished = false;

  public RunIntakeCommand(DoubleSupplier speed, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.led = LEDSubsystem.getInstance();
    intakeSpeed = speed;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
   // SmartDashboard.putBoolean("", finished)

    if (intake.tripLimit()) {
      intake.runIntake(0);
      led.addColorBack(led.PICKED_UP);
      finished = true;
    } else {
      intake.runIntake(intakeSpeed.getAsDouble());
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}