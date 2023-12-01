// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;

//import frc.robot.commands.ExampleCommand;

import frc.robot.subsystems.Yoshi;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drive = Drivetrain.getInstance();
  private final Yoshi m_yoshi = Yoshi.getInstance();
  private final Carriage m_carriage = Carriage.getInstance();
  private final ElevatorLift m_elevator = ElevatorLift.getInstance();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
    OperatorConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Left Triggger HAS to be Infeed (Gabe's law of Eclipsedynamics)
    // Right Trigger HAS to be speedy go fast button (Everybody's law of
    // zoomyrobo-dynamics)

    // Right Bumper is Outfeed (Eric and Zeke's choice)

  

    m_driverController.y().onTrue(m_yoshi.runSwitchBladeForward()).onFalse(m_yoshi.stopSwitchBlade());
    m_driverController.x().onTrue(m_yoshi.runSwitchBladeBackward().until(m_yoshi.supplier()))
        .onFalse(m_yoshi.stopSwitchBlade());

    m_driverController.leftBumper().onTrue(m_carriage.runCarriageIn()).onFalse(m_carriage.stopCarriage());
    m_driverController.leftBumper().onTrue(m_yoshi.runInFeed()).onFalse(m_yoshi.stopInFeed());
    
    m_driverController.rightBumper().onTrue(m_carriage.runCarriageOut()).onFalse(m_carriage.stopCarriage());
    m_driverController.rightBumper().onTrue(m_yoshi.runOutFeed()).onFalse(m_yoshi.stopInFeed());

    

    m_operatorController.x().onTrue(m_elevator.stopElevator());
    m_operatorController.y().onTrue(m_elevator.elevatorRunToPositionCommand60());
    m_operatorController.b().onTrue(m_elevator.elevatorRunToPositionCommand40());
    m_operatorController.a().onTrue(m_elevator.elevatorRunToPositionCommand20());


    m_drive.setDefaultCommand(
        // Get Joystick Axis for Left and Right Sticks (This is in terms of Arcade
        // drive)
        m_drive.driverobot(() -> -m_driverController.getLeftY(), () -> m_driverController.getRightX()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            AutoConstants.ksVolts,
            AutoConstants.kvVoltSecondsPerMeter,
            AutoConstants.kaVoltSecondsSquaredPerMeter),
        AutoConstants.kDriveKinematics,
        10);

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(AutoConstants.kDriveKinematics)
        .addConstraint(autoVoltageConstraint);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        Pose2d(3, 0, new Rotation2d(0)),
        config);
    m_drive.resetOdometry(exampleTrajectory.getInitialPose());

    RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      m_drive::getPose2d,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(
        AutoConstants.ksVolts,
        AutoConstants.kvVoltSecondsPerMeter,
        AutoConstants.kaVoltSecondsSquaredPerMeter
      ), 
      AutoConstants.kDriveKinematics,
      m_drive::getWheelSpeeds,
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      new PIDController(AutoConstants.kPDriveVel, 0, 0),
      m_drive::tankDriveVolts,
      m_drive);

    return null;

  }

  private Pose2d Pose2d(int i, int j, Rotation2d rotation2d) {
    return null;
  }
}
