package frc.robot.commands.AutoCommands.ZPathPlannerAuto.BlueLeft;

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
 *  alignment for this leg is LEFT from BLUE driverstation point of view 
 *  this start position assumes last leg drove away from 
 *  Reef E position as defined in PathPlanner application
*/
public class BlueLLeg4 extends SequentialCommandGroup {

        public BlueLLeg4(Swerve s_Swerve, boolean reversed) {

                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

                // All units in meters.
                Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
                                // taken from Path: BlueR_leg4_toC_mirror (BLUE-left)
                                new Pose2d(1.6080683624801264, 7.354405574562799, new Rotation2d(-0.9424777960769379)),
                                List.of(
                                        new Translation2d(1.830512349159914, 7.083242072698825),
                                        new Translation2d(2.0881461888415886, 6.778551619455477),
                                        // new Translation2d(2.2184400639948594, 6.627425134994839),
                                        new Translation2d(2.481053892292155, 6.326777961430035),
                                        // new Translation2d(2.612936805506801, 6.176854011291701),
                                        new Translation2d(2.8766188306958678, 5.876660103754194),
                                        // new Translation2d(3.0078793953224903, 5.725899750381426),
                                        new Translation2d(3.2678355636388834, 5.421829972318866)),
                                new Pose2d(3.4022920576764744, 5.260396207993615, new Rotation2d(-1.0471975511965976)),
                                config);

                var thetaController = new ProfiledPIDController(
                        Constants.AutoConstants.kPThetaController, 0, 0,
                        Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
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
                                swerveControllerCommand);
        }
}
