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
    public class RedRLeg1 extends  SequentialCommandGroup  {
  
        /** Creates a new RedRLeg1. */
  public RedRLeg1(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // taken from BlueLL-leg1E-18-FLIPP_MIRROR.txt (should rename BlueRLeg1-12_toE-FLIP_MIRROR.txt) positions 
    //  All units in meters.
    Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
       // Start here
       new Pose2d( 10.368250399999997, 2.83, new Rotation2d(0.0) ),
       List.of ( 
         new Translation2d( 10.718494444055324, 2.8292934783875623),
         //new Translation2d( 11.110817930928182, 2.8428032493819924),
         new Translation2d( 11.306663355482476, 2.8495659081258626),
         //new Translation2d( 11.702117179644612, 2.83411411281638),
         new Translation2d( 11.901218765024744, 2.7967805131257926),
         //new Translation2d( 12.286094900411275, 2.6545829021406826),
         new Translation2d( 12.467568079814631, 2.5598878455913034)),
       new Pose2d( 12.569532427912042, 2.500650172546843, new Rotation2d(1.0471975511965979) ),
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
