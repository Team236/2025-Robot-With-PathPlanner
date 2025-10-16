// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.commands.AutoCommands.MetricDriveFwdSideTurn;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FieldCentricTargetLeft extends InstantCommand {
  private Swerve s_Swerve;
  private Pose2d robotFieldPose;
  private Pose2d targetFieldPose;
  private double tv;
  private int targetId;
  private Optional<Alliance> alliance = DriverStation.getAlliance();

  /** Creates a new GoToCoralRightLL. */
  public FieldCentricTargetLeft(Swerve swerve) {
    this.s_Swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //if target is seen
    targetId = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); //target id

    if (tv == 1 && Constants.Targeting.REEF_IDS.contains(targetId)) {
   
      robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
 
    //  SmartDashboard.putNumber("x1 robot center: ", robotFieldPose.getX() / 0.0254);
    //  SmartDashboard.putNumber("y1 robot center: ", robotFieldPose.getY()/ 0.0254);
      
      //april tag coordinates
      double x2 = Constants.Targeting.ID_TO_POSE.get(targetId).getX(); //*Math.sin((angle2));
      double y2 = Constants.Targeting.ID_TO_POSE.get(targetId).getY(); //*Math.cos((angle2));
      double angle2 = Constants.Targeting.ID_TO_POSE.get(targetId).getRotation().getRadians();

      //Get the AprilTag pose now, then reset the pose to this value at the end of MetricDriveFwdAndSideAndTurn
      //(after targeting) so that the driving is field oriented after targeting:
      s_Swerve.getTargetPose(new Pose2d(x2, y2, new Rotation2d(angle2)));
      
      //robotFieldPose is from center of robot
      double angle1 = robotFieldPose.getRotation().getRadians();
      double x1 = robotFieldPose.getX() - (Constants.Targeting.DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER*(0.0254)) * Math.cos(angle2) - (Constants.Targeting.DIST_ROBOT_CENTER_TO_LL_SIDEWAYS*(0.0254))*Math.sin((angle2));
      double y1 = robotFieldPose.getY() + (Constants.Targeting.DIST_ROBOT_CENTER_TO_LL_SIDEWAYS*(0.0254))*Math.cos((angle2)) - (Constants.Targeting.DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER*(0.0254)) * Math.sin((angle2));

      y1 += Constants.Targeting.DIST_TAG_LEFT_BRANCH * Math.cos((angle2)) * 0.0254;
      x1 -= Constants.Targeting.DIST_TAG_LEFT_BRANCH * Math.sin((angle2)) * 0.0254;
    
     /*SmartDashboard.putNumber("Target ID", targetId);
      SmartDashboard.putNumber("x1: ", x1 / 0.0254);
      SmartDashboard.putNumber("y1: ", y1/ 0.0254);
      SmartDashboard.putNumber("angle1", Units.radiansToDegrees(angle1));
      SmartDashboard.putNumber("x2: ", x2/ 0.0254);
      SmartDashboard.putNumber("y2: ", y2/ 0.0254);
      SmartDashboard.putNumber("angle2", Units.radiansToDegrees(angle2));
      */
    //Tried adding asProxy to the end to ensure subsystem reqmts for the commands don't confuse the command scheduler
     CommandScheduler.getInstance().schedule(new MetricDriveFwdSideTurn(s_Swerve, false, x1, y1, angle1, x2, y2, angle2));
      
    }
  }
}
