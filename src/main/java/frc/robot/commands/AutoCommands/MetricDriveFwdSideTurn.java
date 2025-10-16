package frc.robot.commands.AutoCommands;

import frc.robot.Constants;
import frc.robot.commands.Targeting.ResetFieldPoseWithTarget;
import frc.robot.subsystems.Swerve;

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

public class MetricDriveFwdSideTurn extends SequentialCommandGroup {
  //Pass in forward X distance (inches, positive), sideways distance (inches), and turn angle (degrees)
  //X (fwdDist) always positive, so pass in false for "reversed" in Container when command is called
    //X (fwdDist) MUST BE POSITIVE! 
  //Pass in false for "reversed" in Container when command is called
  
    public MetricDriveFwdSideTurn(Swerve s_Swerve, boolean reversed, double fwdDist1, double sideDist1, double turnAngle1, double fwdDist2, double sideDist2, double turnAngle2) {
        //Must pass in current camera pose from LL, and current AprilTag pose from LL (meters, radians)
        //Add or subtract ~6.5 inches from sideDist2 in order to center LL on a branch later in FieldCentricTarget commands
        System.out.println("x1 : " + fwdDist1);
        System.out.println("x2 : " + fwdDist2);
        System.out.println("y1 : " + sideDist1);
        System.out.println("y2 : " + sideDist2);
        System.out.println("angle1 : " + turnAngle1);
        System.out.println("angle2 : " + turnAngle2);
        double deltaFwd = fwdDist2-fwdDist1;
        double deltaSide = sideDist2 - sideDist1;
        double deltaAngle = turnAngle2- turnAngle1;

        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
             new Pose2d(fwdDist1, sideDist1, new Rotation2d((turnAngle1))),
            // Pass through these interior waypoints
            List.of(
                   new Translation2d((fwdDist1+0.05*deltaFwd), (sideDist1+0.05*deltaSide)), 
                   new Translation2d((fwdDist1+0.1*deltaFwd), (sideDist1+0.1*deltaSide)),
                   new Translation2d((fwdDist1+0.15*deltaFwd), (sideDist1+0.15*deltaSide)),
                   new Translation2d((fwdDist1+0.2*deltaFwd), (sideDist1+0.2*deltaSide)), 
                   new Translation2d((fwdDist1+0.25*deltaFwd), (sideDist1+0.25*deltaSide)),
                   new Translation2d((fwdDist1+0.3*deltaFwd), (sideDist1+0.3*deltaSide)),
                   new Translation2d((fwdDist1+0.35*deltaFwd), (sideDist1+0.35*deltaSide)), 
                   new Translation2d((fwdDist1+0.4*deltaFwd), (sideDist1+0.4*deltaSide)),
                   new Translation2d((fwdDist1+0.45*deltaFwd), (sideDist1+0.45*deltaSide)), 
                   new Translation2d((fwdDist1+0.5*deltaFwd), (sideDist1+0.5*deltaSide)),
                   new Translation2d((fwdDist1+0.55*deltaFwd), (sideDist1+0.55*deltaSide)),
                   new Translation2d((fwdDist1+0.6*deltaFwd), (sideDist1+0.6*deltaSide)), 
                   new Translation2d((fwdDist1+0.65*deltaFwd), (sideDist1+0.65*deltaSide)),
                   new Translation2d((fwdDist1+0.7*deltaFwd), (sideDist1+0.7*deltaSide)),
                   new Translation2d((fwdDist1+0.75*deltaFwd), (sideDist1+0.75*deltaSide)), 
                   new Translation2d((fwdDist1+0.8*deltaFwd), (sideDist1+0.8*deltaSide)),
                   new Translation2d((fwdDist1+0.85*deltaFwd), (sideDist1+0.85*deltaSide)), 
                   new Translation2d((fwdDist1+0.9*deltaFwd), (sideDist1+0.9*deltaSide)),
                   new Translation2d((fwdDist1+0.95*deltaFwd), (sideDist1+0.95*deltaSide))
                   ),  
            // End here
            new Pose2d(fwdDist2, sideDist2, new Rotation2d((turnAngle2 + Math.PI))), //add 180 because target and robot are facing opposite directions
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
           swerveControllerCommand, //TODO try removing this as last ditch effort to get it to work
           new ResetFieldPoseWithTarget(s_Swerve)
        );
    }
}