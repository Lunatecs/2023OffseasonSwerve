// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMoveCommand extends PIDCommand {
  /** Creates a new AutoMoveCommand. */
  private SwerveDrive drive;
  private double distance;
  private double kP;

  public AutoMoveCommand(SwerveDrive drive, double distance, double kP) {
    super(
        // The controller that the command will use
        new PIDController(kP, 0, 0),
        // This should return the measurement
        () -> drive.getFLDriveEncoder() * ((2048*6.75)/(4*Math.PI)),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          // Use the output here
          if (output < 0) {
            drive.stop();
          }

          drive.drive(new Translation2d(output, 0).times(Constants.Swerve.maxSpeed), 0, true, false);
        });
        this.drive = drive;
        this.distance = distance;
        this.kP = kP;
        addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
