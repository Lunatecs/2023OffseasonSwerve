// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSinglePathTest extends SequentialCommandGroup {
  /** Creates a new AutoPathPlannerTest. */
  public AutoSinglePathTest(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake, SwerveDrive swerve) {
    addCommands(
      new AutoDeliverTopConeCommand(elevator, arm, wrist, intake),
      new InstantCommand(() -> intake.runIntake(-.55), intake),
      new SetWristAngleCommand(wrist, Constants.WristConstants.GROUND_INTAKE_CUBE),
      new AutoPath(swerve, "Path Test Copy"),
      new AutoDeliverTopConeCommand(elevator, arm, wrist, intake)
    );
  }
}