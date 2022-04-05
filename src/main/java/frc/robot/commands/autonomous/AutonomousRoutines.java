package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;

import frc.robot.subsystems.Drive;

import frc.robot.subsystems.NavXGyro;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import java.util.List;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.RobotContainer;

public class AutonomousRoutines {
    private Drive _drive;

    private NavXGyro _navxGyro;

    RobotContainer robot;

    public AutonomousRoutines(Drive drive,  NavXGyro navXGyro) {
        this._drive = drive;

        //this._targetVision = targetVision;
        this._navxGyro = navXGyro;

    }

    public Command DoNothing() {
        return new DoNothingCommand();
    }

    public Command testMove() {
        return new SequentialCommandGroup(
            new AutonDriveDistanceCommand(this._drive, 30, 0.0, 0.4, 0.0, true)
        );
    }

    public Command testRotate() {
        return new SequentialCommandGroup(
            new RotateCommand(this._drive, 30, this._navxGyro)
        );
    }

    public Command testGoToPoint (){
        return new SequentialCommandGroup(
            new ResetOdometry(this._drive, new Pose2d(0, 0, new Rotation2d(0))),
            new GoToPoint(new Translation2d(2.0, 0), Rotation2d.fromDegrees(0), this._drive)
        );
    }

    public Command testAutoMove(){
         // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            // List.of(
            //         new Translation2d(1, 0),
            //         new Translation2d(1, -1)),
            // new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            // trajectoryConfig);
            List.of(
            // new Translation2d(-1, 0),
            // new Translation2d(-1, -3),
            // new Translation2d(-7.0, -3),
            // new Translation2d(-7.0, 0.6)),
            // new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Translation2d(1,0),
            new Translation2d(1,1)),
            new Pose2d(2, 1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);

            // 3. Define PID controllers for tracking trajectory
            PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
            PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
            ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);

            // 4. Construct command to follow trajectory
            SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                //swerveSubsystem::getPose,
                _drive::getPose,
                Constants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                _drive::setModuleStates,
                _drive);

            // SwerveControllerCommand testController = new SwerveControllerCommand(
            //   trajectory, 
            //   _drive::getPose, 
            //   Constants.kDriveKinematics, 
            //   xController, 
            //   yController, 
            //   thetaController, 
            //   desiredRotation, 
            //   _drive::setModuleStates,
            //   _drive);


            // 5. Add some init and wrap-up, and return everything

            return new SequentialCommandGroup(
                new InstantCommand(() -> _drive.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> _drive.stopModules()));

    }


    



    public Command getCenterRotateFireAndMove(){
        return new SequentialCommandGroup(
            new AutonDriveDistanceCommand(this._drive, 30, 0.4, 0.0, 0.0, false),
            new RotateCommand(this._drive, 90, this._navxGyro),

            new AutonDriveDistanceCommand(this._drive, 10,0.0, 0.4, 0.0, true)
        );
    }

    public Command getLeftRotateFireAndMove(){
        return new SequentialCommandGroup(
            new RotateCommand(this._drive, 30, this._navxGyro),
            new AutonDriveDistanceCommand(this._drive, 25, 0.0, 0.4, 0.0, false),

            new AutonDriveDistanceCommand(this._drive, 10 ,0.0, 0.4, 0.0, true)
        );
    }

    public Command getRightRotateFireAndMove(){
        return new SequentialCommandGroup(
            new RotateCommand(this._drive, 70, this._navxGyro),
            new AutonDriveDistanceCommand(this._drive, 30, 0.0, -0.4, 0.0, false),

            new AutonDriveDistanceCommand(this._drive, 15 ,0.0, -0.4, 0.0, true)
        );
    }

 
}