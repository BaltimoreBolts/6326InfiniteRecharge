/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/*THIS PID CONTROLLER NEEDS WORK JMK*/
public class Shooter extends SubsystemBase {
  CANSparkMax SMotorChip;
  CANSparkMax SMotorDale;
  double shooterMotorSpeed = 0;

  private CANPIDController shooterPID;
  double pVal = 1.0; // These may be terrible default values! DRRM 
  double iVal = 0.5; // These may be terrible default values! DRRM
  double dVal = 0.5; // These may be terrible default values! DRRM
  double pidShooterSpeed = 0.0;
  CANEncoder ShooterEncoder;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    SMotorChip = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_CHIP, MotorType.kBrushed);
    SMotorDale = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DALE, MotorType.kBrushed);

    //Restore factory defaults (not necessary but let's do it anyway)
    SMotorChip.restoreFactoryDefaults();
    SMotorDale.restoreFactoryDefaults();

    shooterPID = SMotorChip.getPIDController();
    ShooterEncoder = SMotorChip.getEncoder();

    // Prints the initial PID values to smart dashboard
    SmartDashboard.putNumber("Current pVal = ", pVal);
    SmartDashboard.putNumber("Current iVal = ", iVal);
    SmartDashboard.putNumber("Current dVal = ", dVal);
    SmartDashboard.putNumber("Calculated PID = ", pidShooterSpeed);
    SmartDashboard.putNumber("Shooter Motor Speed = ", shooterMotorSpeed);
    SmartDashboard.putBoolean("PID or Value:", true); // Set to true for using "Shooter Motor Speed" to control shooter speed
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder positions",ShooterEncoder.getPosition());
    //PIDTuner(); // Comment this out once we figure out our PID values.
  }

  public void PIDTuner() {
    double pTemp = 0;
    double iTemp = 0;
    double dTemp = 0;
    double motorShooterSpeed = 0;
    boolean pidOrValue;

    pTemp = SmartDashboard.getNumber("Current pVal = ", -1);
    iTemp = SmartDashboard.getNumber("Current iVal = ", -1);
    dTemp = SmartDashboard.getNumber("Current dVal = ", -1);
    motorShooterSpeed = SmartDashboard.getNumber("Shooter Motor Speed = ", -1); 
    pidOrValue = SmartDashboard.getBoolean("PID or Value:", true);

    // We don't want to set our constants to a negative value so lets prevent that
    if (pTemp >= 0 ) {
      pVal = pTemp;
    }

    if (iTemp >= 0 ) {
      iVal = iTemp;
    }

    if (dTemp >= 0 ) {
      dVal = dTemp;
    }
    shooterPID.setP(pVal);
    shooterPID.setI(iVal);
    shooterPID.setD(dVal);

    if (pidOrValue) {
      SMotorChip.set(motorShooterSpeed);
      SMotorDale.set(-motorShooterSpeed);
    } else {
      // Use PID value
      double setPoint = 0;
      shooterPID.setReference(setPoint, ControlType.kVelocity);
    
      SmartDashboard.putNumber("SetPoint", setPoint);
      SmartDashboard.putNumber("ProcessVariable", ShooterEncoder.getVelocity());
    }
  }


}
