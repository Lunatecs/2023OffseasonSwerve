// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.commands.AutoPath;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EventTriggerPathTest extends SequentialCommandGroup {
  /** Creates a new EventTriggerPathTest. */
  SwerveDrive swerve;
  AutoPath path;
  public EventTriggerPathTest(SwerveDrive swerve, String pathName, HashMap<String, Command> eventMap, ElevatorSubsystem elevator, ArmSubsystem arm, WristSubsystem wrist, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.swerve = swerve;
    path = new AutoPath(swerve, pathName);

    addCommands(
      new AutoDeliverTopConeOnly(intake, wrist, arm, elevator),
      new FollowPathWithEvents(
        path, 
        path.getMarkers(), 
        eventMap
      )
    );
    addRequirements(swerve, intake, wrist, arm, elevator);
  }
}
