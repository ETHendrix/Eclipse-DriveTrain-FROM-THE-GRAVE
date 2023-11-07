// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorLift extends SubsystemBase {
  public TalonSRX m_elevatorMotor;
  public static ElevatorLift m_elevatorInstance;
  /** Creates a new ElevatorLift. */
  public ElevatorLift() {
  m_elevatorMotor = new TalonSRX(7);
  }

  public static ElevatorLift getInstance() {
    return m_elevatorInstance == null ? m_elevatorInstance = new ElevatorLift() : m_elevatorInstance;
}

public Command runElevatorUp() {
  return run(() -> m_elevatorMotor.set(ControlMode.PercentOutput, 0.6));
}

public Command stopElevator() {
  return run(() -> m_elevatorMotor.set(ControlMode.PercentOutput, 0));
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
