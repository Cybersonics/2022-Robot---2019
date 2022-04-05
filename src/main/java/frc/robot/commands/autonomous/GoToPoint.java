// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

import frc.robot.subsystems.Drive;

public class GoToPoint extends CommandBase {
    enum ThetaProfilePolicy
    {
        MATCH_X,
        MATCH_Y,
        MATCH_MAX,
        MANUAL
    };

    RobotContainer robot;
    private Drive _drive;

    TrapezoidProfile xProfile;
    TrapezoidProfile yProfile;
    TrapezoidProfile thetaProfile;

    Translation2d target;
    Rotation2d rotationTarget;

    Timer profileTime = new Timer();

    double distanceToTarget;
    double angleDiff;

    Twist2d initialVelocity = new Twist2d(0, 0, 0);
    Twist2d finalVelocity = new Twist2d(0, 0, 0);

    static final Rotation2d ZERO_DEG = new Rotation2d(0);

    static final TrapezoidProfile.Constraints DEFAULT_LINEAR_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, 2.0);
    static final TrapezoidProfile.Constraints DEFAULT_ANGULAR_CONSTRAINTS = new TrapezoidProfile.Constraints(360, 360*2);
    
    TrapezoidProfile.Constraints xProfileConstraints = DEFAULT_LINEAR_CONSTRAINTS;
    TrapezoidProfile.Constraints yProfileConstraints = DEFAULT_LINEAR_CONSTRAINTS;
    ThetaProfilePolicy thetaPolicy = ThetaProfilePolicy.MATCH_MAX;
    TrapezoidProfile.Constraints thetaConstraints = null;


    public GoToPoint(RobotContainer container, Translation2d target, Rotation2d rotation, Drive drive)
    {
        this.robot = container;
        this.target = target;
        this.rotationTarget = rotation;
        this._drive = drive;
    }

    public GoToPoint(Translation2d target, Rotation2d rotation, Drive drive)
    {
        this.target = target;
        this.rotationTarget = rotation;
        this._drive = drive;
    }

    GoToPoint withXYConstraints(TrapezoidProfile.Constraints constraints) {
        xProfileConstraints = constraints;
        yProfileConstraints = constraints;
        return this;
    }

    GoToPoint withXConstraints(TrapezoidProfile.Constraints constraints) {
        xProfileConstraints = constraints;
        return this;
    }

    GoToPoint withYConstraints(TrapezoidProfile.Constraints constraints) {
        yProfileConstraints = constraints;
        return this;
    }
    
    GoToPoint withThetaConstraints(TrapezoidProfile.Constraints constraints) {
        thetaConstraints = constraints;
        thetaPolicy = ThetaProfilePolicy.MANUAL;
        return this;
    }
    
    GoToPoint withThetaPolicy(ThetaProfilePolicy policy) {
        thetaPolicy = policy;
        return this;
    }

    double getTimeFromPolicy() {
        if (thetaPolicy == ThetaProfilePolicy.MATCH_X)
        {
            return xProfile.totalTime();
        }
        if (thetaPolicy == ThetaProfilePolicy.MATCH_Y)
        {
            return yProfile.totalTime();
        }
        if (thetaPolicy == ThetaProfilePolicy.MATCH_MAX)
        {
            return Math.max(xProfile.totalTime(), yProfile.totalTime());
        }
        return 0;
    }

    double maxNonNanTime(double... times)
    {
        double max = 0;
        for ( var time : times )
        {
            if ( time > max && !Double.isNaN(time) )
            {
                max = time;
            }
        }
        return max;
    }

    @Override
    public void initialize() {
        
        // robot.swerveDrive.disableRamping();
        this._drive.disableRamping();
        var robotPose = this._drive.getPose();
        xProfile = new TrapezoidProfile(xProfileConstraints, new State(target.getX(), finalVelocity.dx), new State(robotPose.getX(), initialVelocity.dx));
        yProfile = new TrapezoidProfile(yProfileConstraints, new State(target.getY(), finalVelocity.dy), new State(robotPose.getY(), initialVelocity.dy));

        if ( thetaPolicy != ThetaProfilePolicy.MANUAL || thetaPolicy == null )
        {
            var totalRotation = Math.abs(robotPose.getRotation().getDegrees() - rotationTarget.getDegrees());

            var totalRotationDegSec = totalRotation / getTimeFromPolicy();
            if ( Double.isNaN(totalRotationDegSec))
            {
                totalRotationDegSec = DEFAULT_ANGULAR_CONSTRAINTS.maxVelocity;
            }

            thetaConstraints = new TrapezoidProfile.Constraints(totalRotationDegSec, DEFAULT_ANGULAR_CONSTRAINTS.maxAcceleration);
        }
        thetaProfile = new TrapezoidProfile(thetaConstraints, new State(rotationTarget.getDegrees(), 0), new State(robotPose.getRotation().getDegrees(), 0));
        profileTime.reset();
        profileTime.start();
    }

    @Override
    public void execute()
    {
        final var robotPose = this._drive.getPose();
        final var xProfilePoint = xProfile.calculate(profileTime.get());
        final var yProfilePoint = yProfile.calculate(profileTime.get());
        final var thetaProfilePoint = thetaProfile.calculate(profileTime.get());
        final var thisTarget = new Translation2d(xProfilePoint.position, yProfilePoint.position);
        final var thisTargetAngle = Rotation2d.fromDegrees(thetaProfilePoint.position);

        final var fieldTargetPose = new Pose2d(thisTarget, ZERO_DEG);

        final var diff = fieldTargetPose.minus(robotPose);
        distanceToTarget = diff.getTranslation().getNorm();

        angleDiff = robotPose.getRotation().minus(thisTargetAngle).getDegrees();

        var feedforwardVector = new Translation2d(xProfilePoint.velocity, yProfilePoint.velocity);
        feedforwardVector = feedforwardVector.rotateBy(robotPose.getRotation().unaryMinus());

        var xPow = diff.getX() * AutoConstants.kGoToPointLinearP + feedforwardVector.getX() * AutoConstants.kGoToPointLinearF;
        var yPow = diff.getY() * AutoConstants.kGoToPointLinearP + feedforwardVector.getY() * AutoConstants.kGoToPointLinearF;
        var thetaPow = angleDiff * AutoConstants.kGoToPointAngularP + thetaProfilePoint.velocity * AutoConstants.kGoToPointAngularF;
        // this._drive.swerveDrive(-yPow, xPow, thetaPow, 1);
        this._drive.processInput(-yPow, xPow, thetaPow, false);
    }

    @Override
    public boolean isFinished()
    {
        var totalTrajectoryLength = maxNonNanTime(xProfile.totalTime(), yProfile.totalTime(), thetaProfile.totalTime());

        if( profileTime.get() < totalTrajectoryLength)
        {
            return false;
        }

        if (profileTime.get() > totalTrajectoryLength + AutoConstants.maxTrajectoryOverrunSeconds )
        {
            return true;
        }

        return Math.abs(distanceToTarget) < AutoConstants.kMaxDistanceMetersError && Math.abs(angleDiff) < AutoConstants.kMaxAngleDegreesError;
    }

    @Override
    public void end(boolean interrupted)
    {
        this._drive.processInput(0, 0, 0, true);
    }
}
