// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDeliverTopConeCommand extends SequentialCommandGroup {
  /** Creates a new AutoDeliverTopConeCommand. */
  public AutoDeliverTopConeCommand(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake) {
    addCommands(
      new InstantCommand(() -> intake.runIntake(-0.15), intake),
      new SetWristAngleCommand(wrist, WristConstants.WRIST_HOME),
      new SetArmExtensionCommand(0, arm),
      new SetEndableElevatorPositionCommand(elevator, ElevatorConstants.MAX_HEIGHT, 0.00006),
      new SetArmExtensionCommand(ArmConstants.TOP_EXTENSION, arm),
      new SetWristAngleCommand(wrist, WristConstants.CONE_SETPOINT),
      new WaitCommand(0.5),
      new InstantCommand(() -> intake.runIntake(.55), intake),
      new WaitCommand(0.5),
      new InstantCommand(() -> intake.runIntake(0), intake),
      new SetOtherLevelsCommand(elevator, arm, wrist, 0, WristConstants.WRIST_HOME, true)
    );

    addRequirements(elevator, arm, wrist, intake);;
  }
}
