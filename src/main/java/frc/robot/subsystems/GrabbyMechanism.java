package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class GrabbyMechanism extends SubsystemBase {
    // private MotorController
    // private MotorController
    private static GrabbyMechanism m_instance;
    private TalonSRX m_leftYoshiInfeed, m_rightYoshiInfeed, m_leftCarriageInfeed, m_rightCarriageInfeed, m_leftSwitchBlade, m_rightSwitchBlade;

    public GrabbyMechanism() {
        // Declare the motors that are in the infeed
       
        m_leftYoshiInfeed = new TalonSRX(11);
        m_rightYoshiInfeed = new TalonSRX(10);
        m_leftCarriageInfeed = new TalonSRX(8);
        m_rightCarriageInfeed = new TalonSRX(9);


        m_leftYoshiInfeed.follow(m_rightYoshiInfeed);
        m_leftCarriageInfeed.follow(m_rightCarriageInfeed);
        m_rightCarriageInfeed.follow(m_rightYoshiInfeed);
       
        m_rightCarriageInfeed.setInverted(true);
        m_leftYoshiInfeed.setInverted(true);

        m_leftSwitchBlade = new TalonSRX(5);
        m_rightSwitchBlade = new TalonSRX(6);

        m_leftSwitchBlade.follow(m_rightSwitchBlade);

    }

    public static GrabbyMechanism getInstance() {
        return m_instance == null ? m_instance = new GrabbyMechanism() : m_instance;
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

    public void runSwitchBladeForward() {
        m_leftSwitchBlade.set(ControlMode.PercentOutput, 0.5);
        m_rightSwitchBlade.set(ControlMode.PercentOutput, 0.5);
    }
    public void stopSwitchBlade() {
        m_rightSwitchBlade.set(ControlMode.PercentOutput, 0);
    }
    
}
