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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NewNewFieldCentricTargetLeft extends SequentialCommandGroup {
  /** Creates a new NewFieldCentricTargetLeft. */
  private Swerve s_Swerve;
  //private Trajectory exampleTrajectory;
  public Trajectory currentTrajectory;
  //private SwerveControllerCommand swerveControllerCommand;
  public SwerveControllerCommand currentSwerveControllerCommand;
  
  public NewNewFieldCentricTargetLeft(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    this.currentTrajectory = s_Swerve.currentTrajectory;
    this.currentSwerveControllerCommand = s_Swerve.currentSwerveControllerCommand;
    

    //This setDefautValues puts values into exampleTrajectory and swerveControllerCommand 
    //(and those same values into currentTrajectory and currentSwerveControllerCommand)
    //(avoids nulls in the case that a target is not seen)
    //But it does not call the swerveControllerCommand so nothing should move
    //I added two waypoints vice one, since I think 2 waypoints are needed
    //Should try deleting the line below and see if command still works
    s_Swerve.setDefaultValues();
    //this.setDefaultValues();  
   // SmartDashboard.putNumber("pose X after default: ", s_Swerve.getPose().getX());
   // SmartDashboard.putNumber("pose Y after default: ", s_Swerve.getPose().getY());
    s_Swerve.setupValues();

    addCommands( 
    //new WaitCommand(0.3),
    //new InstantCommand (s_Swerve::setupValues, s_Swerve),
    //new InstantCommand(() -> this.setupValues()),//Needs to be command here, or method above ok???

    //new InstantCommand(() -> s_Swerve.setPose(exampleTrajectory.getInitialPose())),
    new InstantCommand(() -> s_Swerve.setPose(currentTrajectory.getInitialPose())).withTimeout(1),

    //swerveControllerCommand,
    currentSwerveControllerCommand.withTimeout(5),  

    //Why not make this an instant command also, using the method resetFldPoseWithTarget from s_Swerve?
    new ResetFieldPoseWithTarget(s_Swerve).withTimeout(1)

    );
  }


}
 