// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.SetPointSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetOtherLevelsCommand extends SequentialCommandGroup {
  /** I HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDSI HATE NAMING COMMANDS */
  public SetOtherLevelsCommand(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist, double setpoint, double wristSetpoint, double elevatorP) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmExtensionCommand(0, arm),  
      new SetWristAngleCommand(wrist, WristConstants.WRIST_HOME));
      if(setpoint == ElevatorConstants.BOTTOM) {
        addCommands(new SetElevatorToZeroCommand(elevator));
      }else {
        addCommands(new SetEndableElevatorPositionCommand(elevator, setpoint, elevatorP));//0.00006
      }
      addCommands(
      new ElevatorBottomCommand(setpoint, elevator),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new SetWristAngleCommand(wrist, wristSetpoint),
          new WristBrakeCommand(new SetPointSupplier(), wrist)
        ),
        new LockElevatorCommand(new SetPointSupplier(), elevator),
        new LockArmCommand(arm, new SetPointSupplier())
      )
    );

    addRequirements(elevator, arm, wrist);
  }

  /*addCommands(
      new SetArmExtensionCommand(0, arm),  
      new SetWristAngleCommand(wrist, WristConstants.WRIST_HOME),
      new SetEndableElevatorPositionCommand(elevator, setpoint, elevatorP),//0.00006
      new ElevatorBottomCommand(setpoint, elevator),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new SetWristAngleCommand(wrist, wristSetpoint),
          new WristBrakeCommand(new SetPointSupplier(), wrist)
        ),
        new LockElevatorCommand(new SetPointSupplier(), elevator),
        new LockArmCommand(arm, new SetPointSupplier())
      )
    ); */

  public SetOtherLevelsCommand(ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist, double setpoint, double wristSetpoint, boolean endable) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmExtensionCommand(0, arm),  
      new SetWristAngleCommand(wrist, WristConstants.WRIST_HOME),
      new SetElevatorToZeroCommand(elevator),
      new SetWristAngleCommand(wrist, wristSetpoint)
    );

    addRequirements(elevator, arm, wrist);
  }

}
