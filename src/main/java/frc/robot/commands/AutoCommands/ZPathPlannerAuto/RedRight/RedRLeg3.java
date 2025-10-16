// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ZPathPlannerAuto.RedRight;

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
    *  alignment for this leg is RIGHT from RED driverstation point of view 
    * 
    *  this start position assumes last leg drove away from 
    *  RED REEF E position as defined in PathPlanner application
    */

    public class RedRLeg3 extends SequentialCommandGroup {
  /** Creates a new RRightLeg3. */
  public RedRLeg3(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // taken from leg2b-FLIP-MIRROR.txt to follow.  All units in meters.
    Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start here
    new Pose2d( 13.476888348084657, 2.2060528849958176, new Rotation2d(1.0471975511965979) ),
        List.of ( 
            new Translation2d( 13.743881932958502, 2.0389853226937475),
            new Translation2d( 14.055697097808213, 1.8813958243798279),
            new Translation2d( 14.23837537743004, 1.8016699997153411),
            new Translation2d( 14.608206988775365, 1.6516821406305162),
            new Translation2d( 14.792584726951727, 1.5746143032239832),
            new Translation2d( 15.152098351657774, 1.3983165178017831),
            new Translation2d( 15.323176644698574, 1.2931368499927993),
            new Translation2d( 15.599780254630128, 1.0771169322416325),
            new Translation2d( 15.71213834414275, 0.9681491614395323)),
        new Pose2d( 15.94018203751987, 0.6974960254372018, new Rotation2d(2.199114857512855) ),
        config );
    

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            legTrajectory,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.setPose(legTrajectory.getInitialPose())),
        swerveControllerCommand
    );
}
}
