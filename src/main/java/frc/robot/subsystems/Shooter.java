/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import java.lang.Math;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GenConstants;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/*THIS PID CONTROLLER NEEDS WORK JMK*/
public class Shooter extends SubsystemBase {
  CANSparkMax SMotorChip;
  CANSparkMax SMotorDale;
  double desiredRPM = 0;
  double motor1ShooterSpeed = 0;

  private CANPIDController shooterPID;
  double kP = 2e-5; 
  double kI = 0; 
  double kD = 0; 
  double kFF = 0.000165;//0.000015;
  CANEncoder ShooterEncoder;

  // Network table for chameleon vision
  NetworkTableInstance table = NetworkTableInstance.getDefault();
  NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("PsThreeCam");
  public NetworkTableEntry targetPose;
  double targetArr[] = new double[]{0,0,0};
  double x,y,angle = 0;


  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    SMotorChip = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_CHIP, MotorType.kBrushed);
    SMotorDale = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DALE, MotorType.kBrushed);
    // Set Dale to follow Chip, but inverted
    SMotorDale.restoreFactoryDefaults();
    SMotorDale.follow(SMotorChip,true);
    //SMotorChip.restoreFactoryDefaults();
    

    ShooterEncoder = SMotorChip.getEncoder(EncoderType.kQuadrature,GenConstants.REV_ENCODER_CPR);
    //Start PID
    shooterPID = SMotorChip.getPIDController();
    shooterPID.setP(kP);
    shooterPID.setI(kI);
    shooterPID.setD(kD);
    shooterPID.setFF(kFF);
    shooterPID.setOutputRange(-1,1);

    // Prints the initial PID values to smart dashboard
    SmartDashboard.putNumber("Current pVal = ", kP);
    SmartDashboard.putNumber("Current iVal = ", kI);
    SmartDashboard.putNumber("Current dVal = ", kD);
    SmartDashboard.putNumber("Current ffVal = ", kFF);
    SmartDashboard.putNumber("Shooter Speed = ", motor1ShooterSpeed);
    SmartDashboard.putNumber("Desired RPM = ",desiredRPM);
    SmartDashboard.putBoolean("Value or SetPID:", true); // Set to true for using "Shooter Motor Speed" to control shooter speed
    SmartDashboard.putNumber("XDist", x);
    SmartDashboard.putNumber("yDist", y);
    SmartDashboard.putNumber("angleDist", angle);
    
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder",ShooterEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Vel",ShooterEncoder.getVelocity());
    targetPose = cameraTable.getEntry("targetPose");
    targetArr = targetPose.getDoubleArray(targetArr);
    x = targetArr[0];
    y = targetArr[1];
    angle = targetArr[2];
    SmartDashboard.putNumber("XDist", x);
    SmartDashboard.putNumber("yDist", y);
    SmartDashboard.putNumber("angleDist", angle);
    
    PIDTuner(); // Comment this out once we figure out our PID values.
  }

  public void PIDTuner() {
    double pTemp = 0;
    double iTemp = 0;
    double dTemp = 0;
    double ffTemp = 0;
    boolean valueOrPID;

    pTemp = SmartDashboard.getNumber("Current pVal = ", 0);
    iTemp = SmartDashboard.getNumber("Current iVal = ", 0);
    dTemp = SmartDashboard.getNumber("Current dVal = ", 0);
    ffTemp = SmartDashboard.getNumber("Current ffVal = ", 0);
    motor1ShooterSpeed = SmartDashboard.getNumber("Shooter Speed = ", 0); 
    desiredRPM = SmartDashboard.getNumber("Desired RPM = ", 0);
    valueOrPID = SmartDashboard.getBoolean("Value or SetPID:", true);
  

    // We don't want to set our constants to a negative value so lets prevent that
    /*if (pTemp >= 0 ) {
      kP = pTemp;
    }

    if (iTemp >= 0 ) {
      kI = iTemp;
    }

    if (dTemp >= 0 ) {
      kD = dTemp;
    }

    if (ffTemp >= 0 ) {
      kFF = ffTemp;
    }*/

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((pTemp != kP)) { shooterPID.setP(pTemp); kP = pTemp; }
    if((kI != iTemp)) { shooterPID.setI(iTemp); kI = iTemp; }
    if((dTemp != kD)) { shooterPID.setD(dTemp); kD = dTemp; }
    if((ffTemp != kFF)) { shooterPID.setFF(ffTemp); kFF = ffTemp; }
   

    if (motor1ShooterSpeed >= 1.0) {
      motor1ShooterSpeed = 1.0;
    } else if (motor1ShooterSpeed <= -1.0) {
      motor1ShooterSpeed = -1.0;
    }

    if (valueOrPID) {
      SMotorChip.set(motor1ShooterSpeed);
      //SMotorDale.set(-motor1ShooterSpeed);
      SmartDashboard.putNumber("Output Chip",SMotorChip.getAppliedOutput());
      SmartDashboard.putNumber("Output Dale",SMotorDale.getAppliedOutput());
    } else {
      // Use PID value
      shooterPID.setReference(desiredRPM, ControlType.kVelocity);
      SmartDashboard.putNumber("Shooter Vel",ShooterEncoder.getVelocity());
      SmartDashboard.putNumber("Output Chip",SMotorChip.getAppliedOutput());
      SmartDashboard.putNumber("Output Dale",SMotorDale.getAppliedOutput());
    }
  }
  public double getNeededVal(double x) {
    double v;
   

    v = (x/Constants.GenConstants.cosTheta)
    *Math.pow(Constants.GenConstants.g/
    (Constants.GenConstants.h*Constants.GenConstants.tanTheta*x-Constants.GenConstants.y0),0.5);

    return v;

  }


}
