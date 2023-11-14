package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private MotorController m_leftMaster, m_leftSlave, m_rightMaster, m_rightSlave;
    private MotorController m_leftGroup, m_rightGroup;
    private DifferentialDrive m_drive;
    private static Drivetrain m_instance;

    public Drivetrain() {
        m_leftMaster = new WPI_TalonSRX(1);
        m_leftSlave = new WPI_TalonSRX(2);
        m_rightMaster = new WPI_TalonSRX(3);
        m_rightSlave = new WPI_TalonSRX(4);

        m_leftGroup = new MotorControllerGroup(m_leftMaster, m_leftSlave);
        m_rightGroup = new MotorControllerGroup(m_rightMaster, m_rightSlave);

        m_leftGroup.setInverted(true);

        m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);
    }

    public static Drivetrain getInstance() {
        return m_instance == null ? m_instance = new Drivetrain() : m_instance;
    }

    public Command driverobot(DoubleSupplier stickInputY, DoubleSupplier stickInputX) {
        return run(() -> m_drive.arcadeDrive(stickInputY.getAsDouble(), -stickInputX.getAsDouble()));

    }

    // public Command runInfeed() {
    // return run()
    // }
}