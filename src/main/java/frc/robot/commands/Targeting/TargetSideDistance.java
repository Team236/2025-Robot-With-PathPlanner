// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Targeting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TargetSideDistance extends Command {
// simple ranging control with Limelight.

// Basic targeting data
//tv = hasTarget, Do you have a valid target?
        // 3D Pose Data
        //.getRobotPose_FieldSpace();    // Robot's pose in field space
        //.getCameraPose_TargetSpace();   // Camera's pose relative to tag
        // .getRobotPose_TargetSpace();     // Robot's pose relative to tag
        // .getTargetPose_CameraSpace();   // Tag's pose relative to camera
        //.getTargetPose_RobotSpace();     // Tag's pose relative to robot
        //Below, X is the sideways distance from target, Y is down distance, Z is forward distance
        //3D pose array contains [0] = X, [1] = Y, [2] = Z, [3] = roll, [4] = pitch, [5] = yaw
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kPstrafe = Constants.Targeting.KP_STRAFE;  //kP value for the sideways 
    private double pipeline = 0; 
    private double tv;
    private double standoffSide; // desired horiz distance in inches from camera to target; pass into command
    private double poseSide , errorSide;
    private Swerve s_Swerve;    
  
  /** Creates a new TargetSideDistance. */
  public TargetSideDistance(Swerve s_Swerve, double standoffSide) {
    this.s_Swerve = s_Swerve;
    this.standoffSide = standoffSide;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn on the LED,  3 = force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    s_Swerve.zeroHeading(); //added this to fix the targeting going the wrong way
    
   // SmartDashboard.putBoolean("starting tsd", true);
    // TODO swap to LimelightHelpers alternative instead of above methods ?
    // LimelightHelpers.setLEDMode_ForceOn("limelight");
    // LimelightHelpers.setPipelineIndex("limelight", pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if (tv ==1) { //tv =1 means Limelight sees a target

    //poseSide is first element in the pose array - which is sideways distance from center of LL camera to the AprilTag in meters  
    poseSide = LimelightHelpers.getTargetPose_CameraSpace("limelight")[0];
    double finalStandoff = Units.inchesToMeters(standoffSide);  //convert desired standoff from inches to meters
    errorSide = poseSide - finalStandoff; 
    double targetingSidewaysSpeed = errorSide*kPstrafe;
   // SmartDashboard.putNumber("Side to side distance - camera to target, in inches: ", dx/0.0254);
    targetingSidewaysSpeed *= -1.0;  //IS NEEDED
    double strafeVal = targetingSidewaysSpeed;
   
   //This sets Y and rotational movement equal to the value passed when command called (which is joystick value)
   // or try strafeVal and rotationVal = 0 if needed (no rotation or movement in Y directions)
   double translationVal = 0; //MathUtil.applyDeadband(translationSup, Constants.stickDeadband);
   double rotationVal = 0; // MathUtil.applyDeadband(rotationSup, Constants.stickDeadband);
   
   /* Drive */
   s_Swerve.drive(
       new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
       rotationVal * Constants.Swerve.maxAngularVelocity, 
       true,  //true for robot centric
       true //true for open loop (?)
   );
    }

    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(
      new Translation2d(0, 0), 
      0 , 
      true,  //true for robot centric
      true //true for open loop (?)
  );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return (Math.abs(errorSide) < Units.inchesToMeters(0.2));
      return false;
  }
}
