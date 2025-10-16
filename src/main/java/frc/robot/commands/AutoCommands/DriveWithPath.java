package frc.robot.commands.AutoCommands;

import frc.robot.Constants;
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

public class DriveWithPath extends SequentialCommandGroup {
  //Pass in forward X distance (inches, positive), sideways distance (inches), and turn angle (degrees)
  //X (fwdDist) always positive, so pass in false for "reversed" in Container when command is called


    public DriveWithPath(Swerve s_Swerve, boolean reversed) {
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(7.170291487008191,0.8077008574090883, new Rotation2d(3.141592653589793)),
            // Pass through these interior waypoints
	List.of(
		//new Translation2d(7.170291487008191,0.8077008574090883),
        new Translation2d(7.006835109821566,0.8247141290212392),
        //new Translation2d(6.842876084723803,0.8732535003135834),
        //new Translation2d(6.6795651811188455,0.9495683059699576),
        new Translation2d(6.518053168410637,1.0499078806741988),
        //new Translation2d(6.3594908160031185,1.170521559110144),
        //new Translation2d(6.205028893300234,1.3076586759616302),
        //new Translation2d(6.055818169705926,1.457568565912494),
        new Translation2d(5.913009414624137,1.6165005636465726),
        //new Translation2d(5.777753397458811,1.780704003847703),
        //new Translation2d(5.651200887613891,1.9464282211997217),
        //new Translation2d(5.534502654493317,2.1099225503864663),
        new Translation2d(5.4288094675010345,2.2674363260917736),
        //new Translation2d(5.335272096040986,2.41521888299948),
        //new Translation2d(5.255041309517113,2.5495195557934225),
        //new Translation2d(5.162694130974405,2.71646854545881),
        new Translation2d(5.131475259827449,2.7781554431581728)),
        //new Translation2d(5.108000453827693,2.828680509447413),
        //new Translation2d(5.094054325548219,2.863755779625048),
        //new Translation2d(5.117537960654374,2.8621667684629086),
        //new Translation2d(5.117537960654374,2.8621667684629086)),
        //new Translation2d(5.117537960654374,2.8621667684629086)),
    new Pose2d(5.117537960654374,2.8621667684629086, new Rotation2d(2.0943951023931953)),config);
            
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