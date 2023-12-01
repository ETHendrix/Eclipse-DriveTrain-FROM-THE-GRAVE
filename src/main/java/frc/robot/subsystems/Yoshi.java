package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** A.k.a. ヨシ */
public class Yoshi extends SubsystemBase {
    // private MotorController
    // private MotorController
    private static Yoshi m_instance;
    private TalonSRX m_leftYoshiInfeed, m_rightYoshiInfeed, m_leftSwitchBlade, m_rightSwitchBlade;

    public Yoshi() {
        // Declare the motors that are in the infeed
       
        m_leftYoshiInfeed = new TalonSRX(11);
        m_rightYoshiInfeed = new TalonSRX(10);

        m_leftYoshiInfeed.follow(m_rightYoshiInfeed);
       
        m_leftYoshiInfeed.setInverted(true);

        m_leftSwitchBlade = new TalonSRX(5);
        m_rightSwitchBlade = new TalonSRX(6);
        m_leftSwitchBlade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        m_rightSwitchBlade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        m_leftSwitchBlade.follow(m_rightSwitchBlade);

        m_rightSwitchBlade.setInverted(true);
    }

    public static Yoshi getInstance() {
        return m_instance == null ? m_instance = new Yoshi() : m_instance;
    }


    public Command RunCarriageInfeed() {
        return null;
    }

    public Command runInFeed() {
        return run(() -> m_rightYoshiInfeed.set(ControlMode.PercentOutput, 0.5));
    }

    public Command stopInFeed() {
        return run(() -> m_rightYoshiInfeed.set(ControlMode.PercentOutput, 0));
    }

    public Command runOutFeed() {
        return run(() -> m_rightYoshiInfeed.set(ControlMode.PercentOutput, -0.5));
    }

    public Command runSwitchBladeForward() {
        return run(() -> m_rightSwitchBlade.set(ControlMode.PercentOutput, 0.2));
    
    }
    public Command stopSwitchBlade() {
        return runOnce(() -> m_rightSwitchBlade.set(ControlMode.PercentOutput, 0));
    }

    public BooleanSupplier supplier() {
        return () -> m_rightSwitchBlade.isRevLimitSwitchClosed() == 1;
    }

   
    public Command runSwitchBladeBackward() {
        return run(() -> m_rightSwitchBlade.set(ControlMode.PercentOutput, -0.2));
    }
    
}
