// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SequentialElevatorDownWristOutAutoCommand extends SequentialCommandGroup {
  /** Creates a new SequentialElevatorDownWristOutAutoCommand. */
  public SequentialElevatorDownWristOutAutoCommand(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetOtherLevelsCommand(elevator, arm, wrist, 0, WristConstants.WRIST_HOME+2000, true),
      new SetWristAngleAndLockCommand(wrist, WristConstants.GROUND_INTAKE_CUBE)
    );
  }
}
