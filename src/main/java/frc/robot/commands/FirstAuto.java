// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.AutoFromPathPlanner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FirstAuto extends SequentialCommandGroup {
  /** Creates a new FirstAuto. */
  public FirstAuto(SwerveDrive swerve) {
    final AutoFromPathPlanner test = new AutoFromPathPlanner(swerve, "Test", 1, 1, false, true, true);
    addCommands(
      new InstantCommand(() -> swerve.resetModulesToAbsolute()),
      new InstantCommand(() -> SmartDashboard.putNumber("Test", 1)),
      new InstantCommand(() -> swerve.resetOdometryForState(test.getInitialState())),
      test);
  }
}
