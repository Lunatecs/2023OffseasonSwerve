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
public class AutoPath extends SequentialCommandGroup {
  /** Creates a new AutoPath. */
  public AutoPath(SwerveDrive swerve, String pathName) {
    final AutoFromPathPlanner path = new AutoFromPathPlanner(swerve, pathName, 4.2, 3, false, true, true);
    swerve.setYaw(0.0);
    addCommands(
      new InstantCommand(() -> swerve.resetModulesToAbsolute()),
      new InstantCommand(() -> swerve.resetOdometryForState(path.getInitialState())),
      path);
  }

  public AutoPath(SwerveDrive swerve, String pathName, double overrideVelocity, double overrideAcceleration) {
    final AutoFromPathPlanner path = new AutoFromPathPlanner(swerve, pathName, overrideVelocity, overrideAcceleration, false, true, true);
    swerve.setYaw(0.0);
    addCommands(
      new InstantCommand(() -> swerve.resetModulesToAbsolute()),
      new InstantCommand(() -> swerve.resetOdometryForState(path.getInitialState())),
      path);
  }
}
