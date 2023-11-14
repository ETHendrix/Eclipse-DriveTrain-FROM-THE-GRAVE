// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorLift extends SubsystemBase {
  public TalonSRX m_elevatorMotor;
  public static ElevatorLift m_elevatorInstance; // (28510/78.75);
  public static final double NATIVE_UNITS_PER_INCH_CONVERSION = (28510 / 78.75);

  /** Creates a new ElevatorLift. */
  public ElevatorLift() {
    m_elevatorMotor = new TalonSRX(7);

    // Encoder configuration
    m_elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    m_elevatorMotor.setSensorPhase(true);
    m_elevatorMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);

    m_elevatorMotor.config_kP(0, 0.01);
    m_elevatorMotor.config_kI(0, 0);
    m_elevatorMotor.config_kD(0, 0);
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

  public Command elevatorRunToPositionCommand() {
    return run(() -> elevatorRunToPosition(1.5));
  }
  
  public void elevatorRunToPosition(double positionInInches) {
    m_elevatorMotor.set(ControlMode.Position, inchesToNativeUnits(positionInInches));
  }
  private static int inchesToNativeUnits(double positionInInches) {
		int nativeUnits = (int)(positionInInches * NATIVE_UNITS_PER_INCH_CONVERSION);
		return nativeUnits;
	}

  private static double nativeUnitsToInches(double nativeUnitsMeasure) {
		double positionInInches = nativeUnitsMeasure / NATIVE_UNITS_PER_INCH_CONVERSION;
		return positionInInches;
	}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator encoder pos", nativeUnitsToInches(m_elevatorMotor.getSelectedSensorPosition()));
    // This method will be called once per scheduler run
  }
}
