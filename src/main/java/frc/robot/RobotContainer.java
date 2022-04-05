// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveCommand;

import frc.robot.commands.autonomous.AutonomousRoutines;

import frc.robot.subsystems.Drive;

import frc.robot.subsystems.NavXGyro;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  //public static Drive _drive = Drive.getInstance(Constants.ROBOT_WIDTH, Constants.ROBOT_LENGTH);
  public static NavXGyro _gyro = NavXGyro.getInstance();
  
  public static Drive _drive = Drive.getInstance(_gyro);

  //public static TargetVision _targetVision = TargetVision.getInstance();

  // Controllers
  public XboxController opController;
  public XboxController driveController;
  public Joystick leftStick;
  public Joystick rightStick;

  // A chooser for autonomous commands
  private final AutonomousRoutines _autonRoutines = new AutonomousRoutines(_drive, _gyro);
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public RobotContainer() {
    // Set up auton selector
    m_chooser.setDefaultOption("Do Nothing", _autonRoutines.DoNothing());
    // m_chooser.addOption("testMove", _autonRoutines.testMove());
    // m_chooser.addOption("testRotate", _autonRoutines.testRotate());
    // m_chooser.addOption("testShooter", _autonRoutines.testShooter());
    // m_chooser.addOption("testIndexer", _autonRoutines.testIndexer());
    m_chooser.addOption("Center Shoot and Move", _autonRoutines.getCenterRotateFireAndMove());
    m_chooser.addOption("Left Shoot and Move", _autonRoutines.getLeftRotateFireAndMove());
    m_chooser.addOption("Right Shoot and Move", _autonRoutines.getRightRotateFireAndMove());
    m_chooser.addOption("test Auto Move", _autonRoutines.testAutoMove());
    m_chooser.addOption("Test Go To Point", _autonRoutines.testGoToPoint());
    //m_chooser.addOption("TestLeftComp", _autonRoutines.testRunLeft()); //added at comp

    // Put the chooser on the dashboard
    SmartDashboard.putData(m_chooser);

    opController = new XboxController(Constants.OP_CONTROLLER);
    //driveController = new XboxController(Constants.DRIVE_CONTROLLER);
    leftStick = new Joystick(Constants.LEFT_STICK);
    rightStick = new Joystick(Constants.RIGHT_STICK);

    CommandScheduler.getInstance()
    // .setDefaultCommand(_drive,
    // new DriveCommand(_drive, driveController, _gyro)
    .setDefaultCommand(_drive,
    new DriveCommand(_drive, leftStick, rightStick, _gyro)
    );

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Left xbox joystick Y(up/down)
    // _indexer.setDefaultCommand(new IndexerCommand(_indexer, opController));

    // // Right xbox joystick Y(up/down)
    // _intake.setDefaultCommand(new IntakeCommand(_intake, opController));

    // // Right xbox joystick X(left/right)
    // _turret.setDefaultCommand(new TurretCommand(_turret, opController));

    //Reset NavX
    //new JoystickButton(leftJoy, 7).whenPressed(new ZeroHeadingCommand(_drive, _navXGyro));
    // new JoystickButton(driveController, 3).whenPressed(() -> _gyro.zeroNavHeading());
    new JoystickButton(leftStick, 7).whenPressed(() -> _gyro.zeroNavHeading());

    // Set LB button 
    // new JoystickButton(opController, 6).whenPressed(() -> _launcher.start());
    // new JoystickButton(opController, 6).whenReleased(() -> _launcher.stop());

    // Set A button
    // new JoystickButton(opController, 1).whenPressed(() -> _pneumatics.intakeToggle());
    // // set X Button
    // new JoystickButton(opController, 3).whenPressed(() -> _turret.lowerTurret());
    // // Set Y Button
    // new JoystickButton(opController, 4).whenPressed(() -> _turret.raiseTurret());
    
    // Set Y button
  //   // new JoystickButton(driveController, 4).whenPressed(() -> _pneumatics.climberToggle());
  //   new JoystickButton(rightStick, 3).whenPressed(() -> _pneumatics.climberToggle());
  //  // Set B button
    // new JoystickButton(driveController, 2).whenPressed(() -> _climber.releaseClimber());
    // new JoystickButton(driveController, 2).whenReleased(() -> _climber.stop());
    // new JoystickButton(rightStick, 4).whenPressed(() -> _climber.releaseClimber());
    // new JoystickButton(rightStick, 4).whenReleased(() -> _climber.stop());
    // Set A button
    // new JoystickButton(driveController, 1).whenPressed(() -> _climber.retractClimber());
    // new JoystickButton(driveController, 1).whenReleased(() -> _climber.stop());
    // new JoystickButton(rightStick, 5).whenPressed(() -> _climber.retractClimber());
    // new JoystickButton(rightStick, 5).whenReleased(() -> _climber.stop());


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

//   public Command getAutonomousCommand() {
//     // 1. Create trajectory settings
//     TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
//             AutoConstants.kMaxSpeedMetersPerSecond,
//             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                     .setKinematics(Constants.kDriveKinematics);

//     // 2. Generate trajectory
//     Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//             new Pose2d(0, 0, new Rotation2d(0)),
//             // List.of(
//             //         new Translation2d(1, 0),
//             //         new Translation2d(1, -1)),
//             // new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
//             // trajectoryConfig);
//             List.of(
//               // new Translation2d(-1, 0),
//               // new Translation2d(-1, -3),
//               // new Translation2d(-7.0, -3),
//               // new Translation2d(-7.0, 0.6)),
//               // new Pose2d(0, 0, Rotation2d.fromDegrees(0)),

//               // new Translation2d(-1, 0),
//               // new Translation2d(-1, -1)),
//               // new Pose2d(-2, -1, Rotation2d.fromDegrees(0)),
//               new Translation2d(1.5, 0),
//               new Translation2d(1.5, 1)),
//               new Pose2d(2.5, 1, Rotation2d.fromDegrees(90)),
//       trajectoryConfig);

//     // 3. Define PID controllers for tracking trajectory
//     PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
//     PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
//     ProfiledPIDController thetaController = new ProfiledPIDController(
//             AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     // 4. Construct command to follow trajectory
//     SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//             trajectory,
//             //swerveSubsystem::getPose,
//             _drive::getPose,
//             Constants.kDriveKinematics,
//             xController,
//             yController,
//             thetaController,
//             _drive::setModuleStates,
//             _drive);

//     // SwerveControllerCommand testController = new SwerveControllerCommand(
//     //   trajectory, 
//     //   _drive::getPose, 
//     //   Constants.kDriveKinematics, 
//     //   xController, 
//     //   yController, 
//     //   thetaController, 
//     //   desiredRotation, 
//     //   _drive::setModuleStates,
//     //   _drive);


//     // 5. Add some init and wrap-up, and return everything
//     return new SequentialCommandGroup(
//             new InstantCommand(() -> _drive.resetOdometry(trajectory.getInitialPose())),
//             swerveControllerCommand,
//             new InstantCommand(() -> _drive.stopModules()));
// }
}
