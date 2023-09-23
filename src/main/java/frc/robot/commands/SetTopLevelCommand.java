// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.SetPointSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetTopLevelCommand extends SequentialCommandGroup {
  /** Creates a new SetLevelCommand. */
  public SetTopLevelCommand(ArmSubsystem arm, ElevatorSubsystem elevator, WristSubsystem wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new SetWristAngleCommand(wrist, WristConstants.WRIST_HOME),
    new SetArmExtensionCommand(0, arm),  
    new SetEndableElevatorPositionCommand(elevator, ElevatorConstants.MAX_HEIGHT, 0.00006),
    new SetArmExtensionCommand(ArmConstants.TOP_EXTENSION, arm),
    new ParallelCommandGroup(
      new SequentialCommandGroup(
          new SetWristAngleCommand(wrist, WristConstants.CONE_SETPOINT),
          new WristBrakeCommand(new SetPointSupplier(), wrist)
        ),
      new LockElevatorCommand(new SetPointSupplier(), elevator),
      new LockArmCommand(arm, new SetPointSupplier())
      )
    );
    addRequirements(elevator, arm, wrist);
  }
}
