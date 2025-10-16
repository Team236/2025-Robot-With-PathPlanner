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
public class TargetAngleSide extends Command {
// Basic targeting data
//tx =  Horizontal offset from crosshair to target in degrees
//ty = Vertical offset from crosshair to target in degrees
//ta = Target area (0% to 100% of image)
//tv = hasTarget, Do you have a valid target?
        // 3D Pose Data
        //.getRobotPose_FieldSpace();    // Robot's pose in field space
        //.getCameraPose_TargetSpace();   // Camera's pose relative to tag
        // .getRobotPose_TargetSpace();     // Robot's pose relative to tag
        // .getTargetPose_CameraSpace();   // Tag's pose relative to camera
        //.getTargetPose_RobotSpace();     // Tag's pose relative to robot
         //Below, X is the sideways distance from target, Y is down distance, Z is forward distance
        // 3D pose array contains [0] = X, [1] = Y, [2] = Z, [3] = roll, [4] = pitch, [5] = yaw

        private double standoffSideways; // desired sideways distance in inches from camera to tag; pass into command
        private double errorSide, poseSide, poseAngle;
  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  //In this case, angular velocity will be set proportional to tx (LL to target horizontal offset angle,)
  //forward speed will be proportional to the forward distance between the robot bumper and the Apriltag,
  //and sideways speed will be proportional to side-to-side distance between the center of the LL lens and the center of the AprilTag
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kProtation = Constants.Targeting.KP_ROTATION;//kP value for rotation
    double kPstrafe = Constants.Targeting.KP_STRAFE;  //kP value for the sideways 
    private double pipeline = 0; 
    private double tv;
    
    private Swerve s_Swerve;    
  
  /** Creates a new TargetAllParallel. */
  public TargetAngleSide(Swerve s_Swerve, double standoffSideways) {
    this.s_Swerve = s_Swerve;
    this.standoffSideways = standoffSideways;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // turn on the LED,  3 = force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    s_Swerve.zeroHeading(); //added this to fix the targeting going the wrong way
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    if (tv == 1) { //tv =1 means Limelight sees a target

    //the angle is the error (angle between target and camera)
    poseAngle = LimelightHelpers.getTargetPose_CameraSpace("limelight")[5];
    //SmartDashboard.putNumber("TargetingAngle: ", poseAngle);
    double targetingAngle = poseAngle * kProtation; 
    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngle *= -1.0; 
    double rotationVal = targetingAngle; 

  //poseSide is first element in the pose array - which is sideways distance from center of LL camera to the AprilTag in meters
    poseSide = LimelightHelpers.getTargetPose_CameraSpace("limelight")[0]; 
    double finalSideways = Units.inchesToMeters(standoffSideways);  //convert desired standoff from inches to meters
    errorSide = poseSide - finalSideways; //or add it?
    double targetingSidewaysSpeed = errorSide*kPstrafe;
    targetingSidewaysSpeed *= -1.0;  
    double strafeVal = targetingSidewaysSpeed;

  //This sets forward movement equal to = 0 
   double translationVal = 0; 


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
    //INSERT CODE TO STOP HERE?
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
    //return ( (Math.abs(errorSide) < Units.inchesToMeters(0.25)) &&  (poseAngle < 1));
     return false;

  }
}
