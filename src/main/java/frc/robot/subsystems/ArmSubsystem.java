// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  private final WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.ARM_MOTOR);
  private final DigitalInput armLimitSwitch = new DigitalInput(ArmConstants.LIMIT_SWITCH);
  private PIDController pidController = null;

  public ArmSubsystem() {
    armMotor.configFactoryDefault();
    armMotor.setNeutralMode(NeutralMode.Brake);
    pidController = new PIDController(0.01, 0.0, 0.0);
    pidController.setTolerance(40);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", getArmEncoder());
    SmartDashboard.putBoolean("arm limit switch", getArmLimitSwitch());
    SmartDashboard.putBoolean("Arm: Red = Bottom", armLimitSwitch.get());
    SmartDashboard.putNumber("arm pid output", pidController.calculate(getArmEncoder()));

    if(!armLimitSwitch.get()) {
      this.resetEncoders();
    }

  }

  
  public void setSpeed(double speed) {   //Asumming that motor works the same as the elevator
      armMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getArmEncoder() {
    return armMotor.getSelectedSensorPosition();
  }

  public boolean getArmLimitSwitch() {
    return armLimitSwitch.get();
  }

  public void lockArm() {
    armMotor.set(ControlMode.PercentOutput, 0);
  }
 
  public void resetEncoders() {
    armMotor.setSelectedSensorPosition(0);
  }
}