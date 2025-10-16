// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoCommands.MetricDriveFwdSideTurn;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewFieldCentricTargetLeft extends SequentialCommandGroup {
  /** Creates a new NewFieldCentricTargetLeft. */
  private Swerve s_Swerve;
  private Trajectory exampleTrajectory;
 // public Trajectory currentTrajectory;
  private SwerveControllerCommand swerveControllerCommand;
 // public SwerveControllerCommand currentSwerveControllerCommand;
  
  public NewFieldCentricTargetLeft(Swerve s_Swerve) {
    // this.s_Swerve = s_Swerve; // NOT DONE IN OTHER SEQUENTIAL COMMANDS

  
 
    // s_Swerve.setDefaultValues();    
  
   
    //this.setupValues();  //SPENCER CAN"T THIS BE DONE HERE?  VICE FIRST COMMAND  BELOW
    System.out.println(s_Swerve.driveTargetingValues[1]);
    System.out.println(s_Swerve.driveTargetingValues[2]);
    System.out.println(s_Swerve.driveTargetingValues[3]);
    addCommands(
   
    new InstantCommand(() -> s_Swerve.setupValues()), 

    // new MetricDriveFwdSideTurn(s_Swerve, false, 
    //   s_Swerve.driveTargetingValues[0], 
    //   s_Swerve.driveTargetingValues[1], 
    //   s_Swerve.driveTargetingValues[2], 
    //   s_Swerve.driveTargetingValues[3], 
    //   s_Swerve.driveTargetingValues[4],
    //   s_Swerve.driveTargetingValues[5])

    new InstantCommand(() -> s_Swerve.setPose(s_Swerve.currentTrajectory.getInitialPose())),

    new ProxyCommand(() -> s_Swerve.currentSwerveControllerCommand),

    new ResetFieldPoseWithTarget(s_Swerve)

    );
  }
}

//   public void setupValues() {
//     Pose2d robotFieldPose;
//     Pose2d targetFieldPose;
//     double tv;
//     int targetId;
//     Optional<Alliance> alliance = DriverStation.getAlliance();
  
//     tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //if target is seen
//     targetId = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); //target id

//     SmartDashboard.putNumber("TV: ", tv);

//     if (tv == 1 && Constants.Targeting.REEF_IDS.contains(targetId)) {
   
//       robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
 
//     //  SmartDashboard.putNumber("x1 robot center: ", robotFieldPose.getX() / 0.0254);
//     //  SmartDashboard.putNumber("y1 robot center: ", robotFieldPose.getY()/ 0.0254);
      
//       //april tag coordinates
//       double x2 = Constants.Targeting.ID_TO_POSE.get(targetId).getX(); //*Math.sin((angle2));
//       double y2 = Constants.Targeting.ID_TO_POSE.get(targetId).getY(); //*Math.cos((angle2));
//       double angle2 = Constants.Targeting.ID_TO_POSE.get(targetId).getRotation().getRadians();

//       //Get the AprilTag pose now, then reset the pose to this value at the end of MetricDriveFwdAndSideAndTurn
//       //(after targeting) so that the driving is field oriented after targeting:
//       s_Swerve.getTargetPose(new Pose2d(x2, y2, new Rotation2d(angle2)));
      
//       //robotFieldPose is from center of robot
//       double angle1 = robotFieldPose.getRotation().getRadians();
//       double x1 = robotFieldPose.getX() - (Constants.Targeting.DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER*(0.0254)) * Math.cos(angle2) - (Constants.Targeting.DIST_ROBOT_CENTER_TO_LL_SIDEWAYS*(0.0254))*Math.sin((angle2));
//       double y1 = robotFieldPose.getY() + (Constants.Targeting.DIST_ROBOT_CENTER_TO_LL_SIDEWAYS*(0.0254))*Math.cos((angle2)) - (Constants.Targeting.DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER*(0.0254)) * Math.sin((angle2));

//       y1 += Constants.Targeting.DIST_TAG_LEFT_BRANCH * Math.cos((angle2)) * 0.0254;
//       x1 -= Constants.Targeting.DIST_TAG_LEFT_BRANCH * Math.sin((angle2)) * 0.0254;
      
//      SmartDashboard.putNumber("Target IDDD", targetId);
//       SmartDashboard.putNumber("x1: ", x1 / 0.0254);
//       SmartDashboard.putNumber("y1: ", y1/ 0.0254);
//       SmartDashboard.putNumber("angle1", Units.radiansToDegrees(angle1));
//       SmartDashboard.putNumber("x2: ", x2/ 0.0254);
//       SmartDashboard.putNumber("y2: ", y2/ 0.0254);
//       SmartDashboard.putNumber("angle2", Units.radiansToDegrees(angle2));
      

//       // DRIVE SEGMENT
 
//         double deltaFwd = x2 - x1;
//         double deltaSide = y2 - y1;
//         double deltaAngle = angle2- angle1;

//         boolean reversed = false; 

//         TrajectoryConfig config =
//             new TrajectoryConfig(
//                     Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//                     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                 .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

//         // An example trajectory to follow.  All units in meters.
//         exampleTrajectory =
//         TrajectoryGenerator.generateTrajectory(
//             // Start here
//              new Pose2d(x1, y1, new Rotation2d((angle1))),
//             // Pass through these interior waypoints
//             List.of(
//                    new Translation2d((x1+0.05*deltaFwd), (y1+0.05*deltaSide)), 
//                    new Translation2d((x1+0.1*deltaFwd), (y1+0.1*deltaSide)),
//                    new Translation2d((x1+0.15*deltaFwd), (y1+0.15*deltaSide)),
//                    new Translation2d((x1+0.2*deltaFwd), (y1+0.2*deltaSide)), 
//                    new Translation2d((x1+0.25*deltaFwd), (y1+0.25*deltaSide)),
//                    new Translation2d((x1+0.3*deltaFwd), (y1+0.3*deltaSide)),
//                    new Translation2d((x1+0.35*deltaFwd), (y1+0.35*deltaSide)), 
//                    new Translation2d((x1+0.4*deltaFwd), (y1+0.4*deltaSide)),
//                    new Translation2d((x1+0.45*deltaFwd), (y1+0.45*deltaSide)), 
//                    new Translation2d((x1+0.5*deltaFwd), (y1+0.5*deltaSide)),
//                    new Translation2d((x1+0.55*deltaFwd), (y1+0.55*deltaSide)),
//                    new Translation2d((x1+0.6*deltaFwd), (y1+0.6*deltaSide)), 
//                    new Translation2d((x1+0.65*deltaFwd), (y1+0.65*deltaSide)),
//                    new Translation2d((x1+0.7*deltaFwd), (y1+0.7*deltaSide)),
//                    new Translation2d((x1+0.75*deltaFwd), (y1+0.75*deltaSide)), 
//                    new Translation2d((x1+0.8*deltaFwd), (y1+0.8*deltaSide)),
//                    new Translation2d((x1+0.85*deltaFwd), (y1+0.85*deltaSide)), 
//                    new Translation2d((x1+0.9*deltaFwd), (y1+0.9*deltaSide)),
//                    new Translation2d((x1+0.95*deltaFwd), (y1+0.95*deltaSide))
//                    ),  
//             // End here
//             new Pose2d(x2, y2, new Rotation2d((angle2 + Math.PI))), //add 180 because target and robot are facing opposite directions
//             config);
            
//         var thetaController =
//             new ProfiledPIDController(
//                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         swerveControllerCommand =
//             new SwerveControllerCommand(
//                 exampleTrajectory,
//                 s_Swerve::getPose,
//                 Constants.Swerve.swerveKinematics,
//                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//                 thetaController,
//                 s_Swerve::setModuleStates,
//                 s_Swerve);
// //SPENCER - REMOVE THE ELSE PART BELOW
//               } //else {
//                 //addCommands(); //TODO  not sure if this is needed? worried code might crash if no target is found ==> no commands are added
//              // } 
//   }

//   public void setDefaultValues() {
//     // Pose2d robotFieldPose;
//     // Pose2d targetFieldPose;
//     // double tv;
//     // int targetId;
//     // Optional<Alliance> alliance = DriverStation.getAlliance();
  
//     // tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //if target is seen
//     // targetId = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); //target id

//     // SmartDashboard.putNumber("TV: ", tv);
   
//     //   robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
 
//     // //  SmartDashboard.putNumber("x1 robot center: ", robotFieldPose.getX() / 0.0254);
//     // //  SmartDashboard.putNumber("y1 robot center: ", robotFieldPose.getY()/ 0.0254);
      
//     //   //april tag coordinates
//     //   double x2 = Constants.Targeting.ID_TO_POSE.get(targetId).getX(); //*Math.sin((angle2));
//     //   double y2 = Constants.Targeting.ID_TO_POSE.get(targetId).getY(); //*Math.cos((angle2));
//     //   double angle2 = Constants.Targeting.ID_TO_POSE.get(targetId).getRotation().getRadians();

//     //   //Get the AprilTag pose now, then reset the pose to this value at the end of MetricDriveFwdAndSideAndTurn
//     //   //(after targeting) so that the driving is field oriented after targeting:
//     //   s_Swerve.getTargetPose(new Pose2d(x2, y2, new Rotation2d(angle2)));
      
//     //   //robotFieldPose is from center of robot
//     //   double angle1 = robotFieldPose.getRotation().getRadians();
//     //   double x1 = robotFieldPose.getX() - (Constants.Targeting.DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER*(0.0254)) * Math.cos(angle2) - (Constants.Targeting.DIST_ROBOT_CENTER_TO_LL_SIDEWAYS*(0.0254))*Math.sin((angle2));
//     //   double y1 = robotFieldPose.getY() + (Constants.Targeting.DIST_ROBOT_CENTER_TO_LL_SIDEWAYS*(0.0254))*Math.cos((angle2)) - (Constants.Targeting.DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER*(0.0254)) * Math.sin((angle2));

//     //   y1 += Constants.Targeting.DIST_TAG_LEFT_BRANCH * Math.cos((angle2)) * 0.0254;
//     //   x1 -= Constants.Targeting.DIST_TAG_LEFT_BRANCH * Math.sin((angle2)) * 0.0254;
      
//     //  SmartDashboard.putNumber("Target IDDD", targetId);
//     //   SmartDashboard.putNumber("x1: ", x1 / 0.0254);
//     //   SmartDashboard.putNumber("y1: ", y1/ 0.0254);
//     //   SmartDashboard.putNumber("angle1", Units.radiansToDegrees(angle1));
//     //   SmartDashboard.putNumber("x2: ", x2/ 0.0254);
//     //   SmartDashboard.putNumber("y2: ", y2/ 0.0254);
//     //   SmartDashboard.putNumber("angle2", Units.radiansToDegrees(angle2));
      

//     //   // DRIVE SEGMENT
 
//     //     double deltaFwd = x2 - x1;
//     //     double deltaSide = y2 - y1;
//     //     double deltaAngle = angle2- angle1;

//         boolean reversed = false; 

//         TrajectoryConfig config =
//             new TrajectoryConfig(
//                     Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//                     Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                 .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

//         // An example trajectory to follow.  All units in meters.
//         exampleTrajectory =

//      /*   TrajectoryGenerator.generateTrajectory(
//             s_Swerve.getPose(),
//             // Pass through these interior waypoints
//             List.of(
//               s_Swerve.getPose().getTranslation()
//                    ),  
//             // End here
//             s_Swerve.getPose(),
//             config);
// */ 
//             TrajectoryGenerator.generateTrajectory(
//               s_Swerve.getPose(),
//               // Pass through these interior waypoints
//               List.of(
//                // new Translation2d(s_Swerve.getPose().getTranslation().getX(), s_Swerve.getPose().getTranslation().getY())//,               
//                 new Translation2d(s_Swerve.getPose().getTranslation().getX(), s_Swerve.getPose().getTranslation().getY() + 0.001)//,
//                // new Translation2d(s_Swerve.getPose().getTranslation().getX(), s_Swerve.getPose().getTranslation().getY() + 0.025)
//                      ),  
//               // End here
//               s_Swerve.getPose(),
//               config);
              
            
//         var thetaController =
//             new ProfiledPIDController(
//                 Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         swerveControllerCommand =
//             new SwerveControllerCommand(
//                 exampleTrajectory,
//                 s_Swerve::getPose,
//                 Constants.Swerve.swerveKinematics,
//                 //new PIDController(0, 0, 0),
//                // new PIDController(0, 0, 0),
//                 new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                 new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//                 thetaController,
//                 s_Swerve::setModuleStates,
//                 s_Swerve);

//   }
// }
 