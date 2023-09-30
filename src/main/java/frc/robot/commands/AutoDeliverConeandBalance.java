// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDeliverConeandBalance extends SequentialCommandGroup {
  /** Creates a new AutoDeliverConeandBalance. */
  public AutoDeliverConeandBalance(SwerveDrive swerve, ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake) {
    addCommands(
      //new AutoDeliverTopConeCommand(elevator, arm, wrist, intake),
      //new AutoSwerveMoveCommand(swerve, 86.5, 0.0001),
      new AutoBalanceCommand(swerve)
    );
  }
}
