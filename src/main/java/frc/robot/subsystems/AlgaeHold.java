// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeHold extends SubsystemBase {
  
 //private SparkMax algaeHoldMotor;
  private TalonFX algaeHoldMotor;

//private SparkMaxConfig algaeHoldConfig;
  private TalonFXConfigurator algaeHoldConfig;
  private TalonFXConfiguration algaeHoldTalonConfig;

  private CurrentLimitsConfigs algaeHoldCurrentConfigs;
  private MotorOutputConfigs algaeHoldOutputConfigs;
 

  private DigitalInput algaeHoldLimit;
  private boolean isAHoldException;

  public AlgaeHold() {
    algaeHoldMotor = new TalonFX(Constants.MotorControllers.ID_ALGAE_HOLD);//on the Rio bus, not the usb bus
   // algaeHoldMotor = new SparkMax(Constants.MotorControllers.ID_ALGAE_HOLD, MotorType.kBrushless);
   
  // configure motor
    algaeHoldTalonConfig = new TalonFXConfiguration();
    algaeHoldTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    algaeHoldTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    algaeHoldTalonConfig.CurrentLimits.SupplyCurrentLimit = Constants.MotorControllers.SMART_CURRENT_LIMIT;
    algaeHoldTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeHoldMotor.getConfigurator().apply(algaeHoldTalonConfig);
   
  // algaeHoldConfig = new SparkMaxConfig(); 
  //algaeHoldConfig.inverted(false);
  //algaeHoldConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);
  //algaeHoldMotor.configure(algaeHoldConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    try {
      algaeHoldLimit = new DigitalInput(Constants.AlgaeHold.DIO_AH_LIMIT);
    } catch (Exception e)
    {
      isAHoldException = true;
      SmartDashboard.putBoolean("AlgaeHold Limit switch threw an exception", true);
    }
}

//METHODS Start Here
public boolean isAHoldLimit(){ //Leave normally open
  if (isAHoldException){
    return true; } 
  else{
    return !algaeHoldLimit.get(); //TODO: May need to change back for actual switch (temporarily inverted
  }
 }

 public boolean getAHoldLimit() {
    return !algaeHoldLimit.get(); //TODO: May need to change back for actual switch (temporarily inverted)
 }

  public void stopAlgaeHold(){
    algaeHoldMotor.set(0);
  }

  public void setAlgaeHoldSpeed(double speed1, double speed2)
  {
    if (isAHoldLimit()){
      algaeHoldMotor.set(speed2);
    } else {
      algaeHoldMotor.set(speed1);
    }
    }
    
  public void setAlgaeReleaseSpeed(double speed){ 
    //pass in a negative speed here
    algaeHoldMotor.set(speed);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("AlgaeHold limit:", isAHoldLimit());
  }
}