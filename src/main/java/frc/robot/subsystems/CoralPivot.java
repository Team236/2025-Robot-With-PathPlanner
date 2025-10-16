// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//TODO change to using a brushed motor, with SparkMax controller 
//TODO also change to external encoder (see 2024 front drive motors - 2 DIO channels A/B needed)

public class CoralPivot extends SubsystemBase {
  
  private SparkMax coralPivotMotor;
  private SparkBaseConfig coralPivotConfig;
  //private RelativeEncoder coralPivotEncoder;
  private Encoder coralPivotEncoder;
  private boolean isCoralPivotException;
  public boolean isInitialExtend;
  private double desiredSpeed;
  private DigitalInput CoralLimit;
 

    /** Creates a new CoralPivot. */
    public CoralPivot() {
    coralPivotMotor = new SparkMax(Constants.MotorControllers.ID_CORAL_PIVOT, MotorType.kBrushed);
    // coralPivotEncoder = coralPivotMotor.getEncoder(); 
    coralPivotEncoder = new Encoder(Constants.CoralPivot.DIO_ENC_A, Constants.CoralPivot.DIO_ENC_B);


    coralPivotConfig = new SparkMaxConfig();
    coralPivotConfig.inverted(false); //TODO change to false when using 2024 Robot Tilt motor ID50
    coralPivotConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    coralPivotMotor.configure(coralPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
    try {
      //  This tries to make a new digital input, and if it fails, throws an error 
      CoralLimit = new DigitalInput(Constants.CoralPivot.DIO_LIMIT);
    } catch (Exception e) {
       isCoralPivotException = true;
      SmartDashboard.putBoolean("exception thrown for Coral limit: ", isCoralPivotException);
    }
 
    desiredSpeed = 0;
}

// methods start here

//public double getRawEncoder() {
//return coralPivotEncoder.getRaw();
//}

public double getCoralEncoder() {  //gives encoder reading in Revs
  //return coralPivotEncoder.getPosition(); // if using SparkMax internal encoder
  //for extenal Bourne encoder (512 counts per rev):
  return coralPivotEncoder.getRaw(); //getRaw gets actual count unscaled by the 1, 2 or 4x scale
  //return coralPivotEncoder.get(); //gets count adjusted for the 1, 2 or 4x scale factor
}

public void resetCoralEncoder() {
  coralPivotEncoder.reset();
}

public void stopCoralPivot() {
 // SmartDashboard.putBoolean("set speed of coral pivot motor: ", true);
  coralPivotMotor.set(0);
}

public double getCoralPivotSpeed() {
  return coralPivotMotor.get();
}

public boolean isCoralLimit() {
if (isCoralPivotException) {
  return true;
} else {
  return CoralLimit.get();
}
}

public boolean isFullyExtended() { //revs max is a negative number
  return (getCoralEncoder() <= Constants.CoralPivot.ENC_REVS_MAX);
}

//public boolean atRetractLimit(){
 // if ((getCoralPivotSpeed() >= 0 || desiredSpeed >= 0) && isCoralLimit()){ //positive speed means retracting
   // return true;
   //  } else {
    //  return false;
   // }
   //}

public void setCoralPivotSpeed(double speed) {
  desiredSpeed = speed;
  if (speed <= 0) { //negative speed means extending 
    if (isFullyExtended()) {
      //SmartDashboard.putBoolean("the CP speed negative or 0 and fully extended is true- stop: ", true);
      // if extend limit is tripped or at the maximum desired extension and going out, stop 
        stopCoralPivot();
     }  else {
       // SmartDashboard.putBoolean("the CP speed negative or 0 and fully extended is false- : go", true);
        //extending out but fully extended limit is not tripped, go at commanded speed
       coralPivotMotor.set(speed);
      }
 } 
 else { //speed > 0, so retracting
      if (isCoralLimit()) {
        //SmartDashboard.putBoolean("the CP retract limit is hit- stop: ", true);
        //retract limit is tripped, stop and zero encoder
        stopCoralPivot();
        resetCoralEncoder();
      } else {
        // retract limit is not tripped, go at commanded speed
        //SmartDashboard.putBoolean("the CP retact limit not hit - go: ", true);
        coralPivotMotor.set(speed); 
      }
     }

}

//Begin things that may not be relevant
//these are things that might be useful in the future if we use CANSparkMax PID
//we are not currently using it

//!!!! SPARKMAX PID STUFF - USE SPARKMAX PID, NOT WPILib PID 
//**** CHANGED BACK TO USING WPILib PID ****
//**** due to spurious encoder polarity changes when run multiple autos in a row ****
/*
public void setSetpoint(double encoderRevs) {
  tiltPIDController.setReference(encoderRevs, ControlType.kPosition);
}

public void setP(double kP) {
  tiltPIDController.setP(kP);
}

public void setI(double kI) {
  tiltPIDController.setI(kI);
}

public void setD(double kD) {
  tiltPIDController.setD(kD);
}

public void setFF(double kFF) {
  tiltPIDController.setFF(kFF);
}
*/

@Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Pivot Retract Limit is hit:", isCoralLimit());
    SmartDashboard.putBoolean("Coral Pivot is fully extended: ", isFullyExtended());
    SmartDashboard.putNumber("Coral Pivot Encoder Revolutions ", getCoralEncoder());
   // SmartDashboard.putNumber("coral pivot speed", getCoralPivotSpeed());

  }

}