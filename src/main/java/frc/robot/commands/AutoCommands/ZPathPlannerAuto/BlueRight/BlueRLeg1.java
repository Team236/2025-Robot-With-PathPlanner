// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.ZPathPlannerAuto.BlueRight;

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
    *  alignment for this leg is Right from BLUE driverstation point of view 
    *  robot edge of Bumpers is (18 inches to the LEFT) of center RIGHT side CAGE
    *  this position is aligned with Reef E position as defined in path planner 
   */
  public class BlueRLeg1 extends  SequentialCommandGroup  {
  /** Creates a new RedRLeg1. */
  public BlueRLeg1(Swerve s_Swerve, boolean reversed) {

    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

    // taken from BlueLL-leg1E-18.txt (should rename BlueRLeg1-12_toE.txt) positions  All units in meters.
    Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
       // Start here
       new Pose2d( 7.18, 2.83, new Rotation2d(3.141592653589793) ),
       List.of ( 
         new Translation2d( 6.829755955944673, 2.8292934783875627),
        // new Translation2d( 6.437432469071816, 2.8428032493819924),
         new Translation2d( 6.241587044517521, 2.8495659081258626),
        // new Translation2d( 5.8461332203553855, 2.83411411281638),
         new Translation2d( 5.647031634975254, 2.7967805131257926),
        // new Translation2d( 5.262155499588721, 2.6545829021406826),
         new Translation2d( 5.080682320185366, 2.559887845591304) ),
       new Pose2d( 4.978717972087955, 2.5006501725468424, new Rotation2d(2.0943951023931953) ),
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
