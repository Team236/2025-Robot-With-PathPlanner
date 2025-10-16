// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveFwdAndTurn extends SequentialCommandGroup {
  /** Creates a new DriveFwdAndTurn. */
  public DriveFwdAndTurn(Swerve s_Swerve, boolean reversed, double fwdDist, double turnAngle) {
    //****WAS NOT DRIVINg ENOUGH SIDEWAYS WITHOUT THIS FACTOR!!*****


        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
             new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these interior waypoints
            List.of(
                   new Translation2d(Units.inchesToMeters(0.05*fwdDist), 0), 
                   new Translation2d(Units.inchesToMeters(0.1*fwdDist), 0),
                   new Translation2d(Units.inchesToMeters(0.15*fwdDist), 0),
                   new Translation2d(Units.inchesToMeters(0.2*fwdDist), 0), 
                   new Translation2d(Units.inchesToMeters(0.25*fwdDist),0),
                   new Translation2d(Units.inchesToMeters(0.3*fwdDist), 0),
                   new Translation2d(Units.inchesToMeters(0.35*fwdDist),0), 
                   new Translation2d(Units.inchesToMeters(0.4*fwdDist), 0),
                   new Translation2d(Units.inchesToMeters(0.45*fwdDist),0), 
                   new Translation2d(Units.inchesToMeters(0.5*fwdDist), 0),
                   new Translation2d(Units.inchesToMeters(0.55*fwdDist),0),
                   new Translation2d(Units.inchesToMeters(0.6*fwdDist), 0), 
                   new Translation2d(Units.inchesToMeters(0.65*fwdDist),0),
                   new Translation2d(Units.inchesToMeters(0.7*fwdDist), 0),
                   new Translation2d(Units.inchesToMeters(0.75*fwdDist),0), 
                   new Translation2d(Units.inchesToMeters(0.8*fwdDist),0),
                   new Translation2d(Units.inchesToMeters(0.85*fwdDist),0), 
                   new Translation2d(Units.inchesToMeters(0.9*fwdDist),0),
                   new Translation2d(Units.inchesToMeters(0.95*fwdDist),0)
                   ),  
            // End here
            new Pose2d(Units.inchesToMeters(fwdDist), 0, new Rotation2d(Units.degreesToRadians(turnAngle))),
            config);
            
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
  }
}
