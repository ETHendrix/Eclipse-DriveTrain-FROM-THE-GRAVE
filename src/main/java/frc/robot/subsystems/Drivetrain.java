package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private WPI_TalonSRX m_leftMaster, m_leftSlave, m_rightMaster, m_rightSlave;
    private MotorController m_leftGroup, m_rightGroup;
    private DifferentialDrive m_drive;
    private static Drivetrain m_instance;

    private static final double CODES_PER_REV = 30725.425;
    private static final double ENCODER_ROTATIONS_PER_DEGREE = 46.15 / 3600;
    // private DifferentialDriveOdometry m_odometry;

    public Drivetrain() {
        m_leftMaster = new WPI_TalonSRX(1);
        m_leftSlave = new WPI_TalonSRX(2);
        m_rightMaster = new WPI_TalonSRX(3);
        m_rightSlave = new WPI_TalonSRX(4);

        m_leftMaster.setSensorPhase(true);
        m_leftSlave.setSensorPhase(true);

        m_leftMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0.1));
        m_leftSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0.1));
        m_rightMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0.1));
        m_rightSlave.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 40, 0.1));

        m_leftGroup = new MotorControllerGroup(m_leftMaster, m_leftSlave);
        m_rightGroup = new MotorControllerGroup(m_rightMaster, m_rightSlave);

        m_leftGroup.setInverted(true);

        m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

        configMasterMotors(m_leftMaster);
        configMasterMotors(m_rightMaster);

        // DifferentialDriveOdometry m_odometry = new
        // DifferentialDriveOdometry(m_gyro.getRotation2d(),
        // m_leftMaster.getSelectedSensorPosition(),
        // m_rightMaster.getSelectedSensorPosition(),
        // new Pose2d(0, 0, new Rotation2d()));
    }

    private void configMasterMotors(WPI_TalonSRX m_talon) {
        m_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        m_talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
    }

    private static int inchesToNativeUnits(double positionInInches) {
        int nativeUnits = (int) (positionInInches * CODES_PER_REV / (6 * Math.PI));
        return nativeUnits;
    }

    private static double nativeUnitsToInches(double nativeUnitsMeasure) {
        double positionInInches = nativeUnitsMeasure / CODES_PER_REV * 6 * Math.PI;
        return positionInInches;
    }

    public static Drivetrain getInstance() {
        return m_instance == null ? m_instance = new Drivetrain() : m_instance;
    }

    public Command driverobot(DoubleSupplier stickInputY, DoubleSupplier stickInputX) {
        return run(() -> m_drive.arcadeDrive(stickInputY.getAsDouble(), -stickInputX.getAsDouble()));

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Encoder pos", nativeUnitsToInches(m_leftMaster.getSelectedSensorPosition()));
        SmartDashboard.putNumber("Right Encoder pos", nativeUnitsToInches(m_rightMaster.getSelectedSensorPosition()));
    }
    // public Command runInfeed() {
    // return run()
    // }
}