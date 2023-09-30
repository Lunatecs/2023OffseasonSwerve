// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimeLightSubsystem. */
  public LimeLightSubsystem() {
    CameraServer.startAutomaticCapture(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
