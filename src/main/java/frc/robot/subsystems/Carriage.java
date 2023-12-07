// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Carriage extends SubsystemBase {
  /** Creates a new Carriage. */
  private static Carriage m_carriageInstance;
  private TalonSRX m_leftCarriage, m_rightCarriage;

  private DigitalInput m_carriageLimitSwitch;

  private static final Boolean carriageLimitSwitch = false;

  public Carriage() {

    m_leftCarriage = new TalonSRX(8);
    m_rightCarriage = new TalonSRX(9);

    m_carriageLimitSwitch = new DigitalInput(0);
    m_leftCarriage.follow(m_rightCarriage);

    m_leftCarriage.setInverted(false);
    m_rightCarriage.setInverted(true);
  

  }

  public Command runCarriageIn() {

    return run(() -> m_rightCarriage.set(ControlMode.PercentOutput, 0.5));

  }

  public Command runCarriageOut() {

    return run(() -> m_rightCarriage.set(ControlMode.PercentOutput, -0.5));

  }

  public Command stopCarriage() {

    return run(() -> m_rightCarriage.set(ControlMode.PercentOutput, 0));

  }

  public static Carriage getInstance() {
    return m_carriageInstance == null ? m_carriageInstance = new Carriage() : m_carriageInstance;
  }

  
  public BooleanSupplier isCubeInSupplier() {
    return m_carriageLimitSwitch::get;
    
    
  
} 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Is Cube In?", isCubeInSupplier().getAsBoolean());
  }

}
