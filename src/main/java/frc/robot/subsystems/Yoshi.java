package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU.GeneralStatus;

/** A.k.a. ヨシ */
public class Yoshi extends SubsystemBase {
    // private MotorController
    // private MotorController
    private static Yoshi m_instance;
    private TalonSRX m_leftYoshiInfeed, m_rightYoshiInfeed, m_leftSwitchBlade, m_rightSwitchBlade;

    public static final double NATIVE_UNITS_TO_DEGREES_CONVERSION = (360.0 / 4096.0); // (4096 / 10 / 360);

    public Yoshi() {
        // Declare the motors that are in the infeed

        m_leftYoshiInfeed = new TalonSRX(11);
        m_rightYoshiInfeed = new TalonSRX(10);

        m_leftYoshiInfeed.follow(m_rightYoshiInfeed);

        m_leftYoshiInfeed.setInverted(true);

        m_leftSwitchBlade = new TalonSRX(5);
        m_rightSwitchBlade = new TalonSRX(6);
        m_leftSwitchBlade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyClosed);
        m_rightSwitchBlade.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyClosed);

        m_leftSwitchBlade.follow(m_rightSwitchBlade);

        m_rightSwitchBlade.setInverted(true);


        m_rightSwitchBlade.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        m_rightSwitchBlade.setSensorPhase(false);
        m_rightSwitchBlade.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);

        m_rightSwitchBlade.configMotionCruiseVelocity(4000);
        m_rightSwitchBlade.configMotionAcceleration(4000);

        m_rightSwitchBlade.config_kP(0, 0.45);
        m_rightSwitchBlade.config_kI(0, 0);
        m_rightSwitchBlade.config_kD(0, 4.5);
        m_rightSwitchBlade.config_kF(0, 0);
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

    public Command switchBladeRuntoAngleCommand180() {
        return run(() -> runToAngle(190.0));
    }

    public void runToAngle(double targetAngle) {
        m_rightSwitchBlade.set(ControlMode.MotionMagic, DegreesToNativeUnits(targetAngle));
    }

    private static double nativeUnitsToDegrees(double nativeUnitsMeasure) {
        double positionInDegrees = nativeUnitsMeasure * NATIVE_UNITS_TO_DEGREES_CONVERSION;
        return positionInDegrees;
    }

    private static int DegreesToNativeUnits(double positionInDegrees) {
        int nativeUnits = (int) (positionInDegrees / NATIVE_UNITS_TO_DEGREES_CONVERSION);
        return nativeUnits;
    }

    public double getCurrentRightInfeedPosition() {
        return m_rightSwitchBlade.getSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        if (m_rightSwitchBlade.getSensorCollection().isRevLimitSwitchClosed() == false) {
            m_rightSwitchBlade.setSelectedSensorPosition(0, 0, 0); // zero encoder
            m_rightSwitchBlade.set(ControlMode.PercentOutput, 0); // stop motor
        }

        if (m_rightSwitchBlade.getSensorCollection().isRevLimitSwitchClosed() == false) {
            System.out.println("Switchblades are at home");
        }

        SmartDashboard.putNumber("Right Yoshi encoder pos",
                nativeUnitsToDegrees(m_rightSwitchBlade.getSelectedSensorPosition()));
    }
}
