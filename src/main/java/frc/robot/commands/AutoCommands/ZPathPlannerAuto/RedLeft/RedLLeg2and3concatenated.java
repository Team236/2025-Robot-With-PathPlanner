// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ZPathPlannerAuto.RedLeft;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead 
   https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

   /* 
    *  alignment for this leg is LEFT from RED driverstation point of view 
    *  this start position assumes Reef J position as defined in PathPlanner application
   */
  public class RedLLeg2and3concatenated extends  SequentialCommandGroup  {
  /** Creates a new RedRLeg1. */
  public RedLLeg2and3concatenated(Swerve s_Swerve, boolean reversed) {
    
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // from leg2a-MIRROR positions  All units in meters.
    Trajectory leg2aTrajectory = TrajectoryGenerator.generateTrajectory(
        // Starting from REEF J
        new Pose2d( 4.978717972087955, 5.551251427453158, new Rotation2d(-2.0943951023931953) ),
        // Pass through these interior waypoints
        List.of (
          new Translation2d( 4.649200493122405, 5.7185077870301),
          new Translation2d( 4.282408107413086, 5.775177530099329),
          new Translation2d( 4.0783763450190715, 5.841859991944469)),
        new Pose2d( 4.071362051915341, 5.845848715004183, new Rotation2d(-2.0943951023931953) ),
        config );

    // from leg2a-MIRROR positions  All units in meters.
    Trajectory leg2bTrajectory = TrajectoryGenerator.generateTrajectory(
        // Assumes previous pose moved off of REEF J
        new Pose2d( 4.071362051915341, 5.845848715004183, new Rotation2d(-2.0943951023931953) ),
            List.of ( 
                new Translation2d( 3.8043684670414954, 6.012916277306253),
                // new Translation2d( 3.4925533021917827, 6.170505775620173),
                new Translation2d( 3.309875022569957, 6.25023160028466),
                // new Translation2d( 2.9400434112246323, 6.4002194593694846),
                new Translation2d( 2.75566567304827, 6.4772872967760176),
                // new Translation2d( 2.396152048342224, 6.653585082198218),
                new Translation2d( 2.225073755301423, 6.7587647500072014),
                // new Translation2d( 1.9484701453698698, 6.974784667758368),
                new Translation2d( 1.8361120558572477, 7.083752438560468)),
        new Pose2d( 1.6080683624801264, 7.354405574562799, new Rotation2d(-0.9424777960769379) ),
        config );

    Trajectory combinedTrajectory = leg2aTrajectory.concatenate(leg2bTrajectory);
 
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            combinedTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.setPose(combinedTrajectory.getInitialPose())),
        swerveControllerCommand
    );
}
}
