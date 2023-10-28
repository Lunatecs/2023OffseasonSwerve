// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DevilHornConstants;

public class DevilHornSubsystem extends SubsystemBase {
  private final WPI_TalonFX forkMotor = new WPI_TalonFX(DevilHornConstants.FORK_MOTOR);
  private final Servo dropServo = new Servo(DevilHornConstants.DROP_SERVO);

  public DevilHornSubsystem() {
    forkMotor.configFactoryDefault();
    resetEncoders();
    forkMotor.setNeutralMode(NeutralMode.Brake);
    dropServo.set(0);
  }

  public void setForkSpeed(double speed) {
    forkMotor.set(ControlMode.PercentOutput, speed);
  }

  public void dropForks(boolean drop) {
    if (drop) {
      
      dropServo.set(1);
      System.out.println("Forks Drop");
    }
  }

  public void resetEncoders() {
    forkMotor.setSelectedSensorPosition(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("dropForks Val", dropServo.get());
  }
}
