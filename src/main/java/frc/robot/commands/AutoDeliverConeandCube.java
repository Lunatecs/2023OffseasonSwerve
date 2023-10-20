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
public class AutoDeliverConeandCube extends SequentialCommandGroup {
  /** Creates a new AutoDeliverConeandCube. */
  public AutoDeliverConeandCube(IntakeSubsystem intake, WristSubsystem wrist, ArmSubsystem arm, ElevatorSubsystem elevator, SwerveDrive swerve) {
    addCommands(
      new AutoDeliverTopConeOnly(intake, wrist, arm, elevator),
      new AutoPathandWristLockDeadline(swerve, elevator, arm, wrist, intake),
      new AutoDeliverTopConeCommand(elevator, arm, wrist, intake)
    );
  }
}
