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
  private CANSparkMax SMotorChip;
  private CANSparkMax SMotorDale;
  private double desiredRPM = 0;
  private double motor1ShooterSpeed = 0;

  private CANPIDController shooterPID;
  private double kP = 2e-5; 
  private double kI = 0; 
  private double kD = 0; 
  private double kFF = 0.000165;//0.000015;
  private CANEncoder ShooterEncoder;

  // Network table for chameleon vision
  NetworkTableInstance table = NetworkTableInstance.getDefault();
  NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("PsThreeCam");
  private NetworkTableEntry targetPose;
  private double targetArr[] = new double[]{0,0,0};
  private double x,y,angle = 0;
  private double fudgeFactor = 0;


  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    SMotorChip = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_CHIP, MotorType.kBrushed);
    SMotorDale = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DALE, MotorType.kBrushed);
    SMotorChip.restoreFactoryDefaults();
    SMotorDale.restoreFactoryDefaults();
    SMotorChip.setSmartCurrentLimit(30);
    SMotorDale.setSmartCurrentLimit(30);
    SMotorChip.setIdleMode(CANSparkMax.IdleMode.kCoast);
    SMotorDale.setIdleMode(CANSparkMax.IdleMode.kCoast);

    SMotorChip.burnFlash();
    SMotorDale.burnFlash();
    // Set Dale to follow Chip, but inverted
    SMotorDale.restoreFactoryDefaults();
    SMotorDale.follow(SMotorChip,true);
    
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
    SmartDashboard.putNumber("Calculated velocity", 0);
    SmartDashboard.putNumber("Calculated RPM", 0);
    SmartDashboard.putNumber("fudge Factor", fudgeFactor);
    
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
    SmartDashboard.getNumber("fudge Factor", fudgeFactor);
    
    PIDTuner(); // Comment this out once we figure out our PID values.
    getNeededRPM(x, fudgeFactor);
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
  /*Determine RPM of shooter needed to score power cells in power port 
  ** xdist - distance from power port in m from vision processing
  ** fudgeFactor - multiplication needed to turn velocity into RPM
  */
  public double getNeededRPM(double xDist_m, double fudgeFactor) {

    double vel, RPM;

    // Convert xdist to feet
    double xDist_ft = xDist_m*GenConstants.M_TO_FEET;
   
    vel = (xDist_ft/Constants.GenConstants.COS_ANGLE)
      *Math.pow(-Constants.GenConstants.G_FT_PER_SEC2/
      (Constants.GenConstants.INNER_PORT_HEIGHT_FT-Constants.GenConstants.TAN_ANGLE*xDist_ft
      -Constants.GenConstants.SHOOTER_HEIGHT_FT),0.5);

    RPM = fudgeFactor*vel;

    SmartDashboard.putNumber("Calculated velocity", vel);
    SmartDashboard.putNumber("Calculated RPM", RPM);

    return RPM;

  }

  


}
