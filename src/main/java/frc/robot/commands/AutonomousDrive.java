/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;



public class AutonomousDrive extends CommandBase {
  /**
   * Creates a new Autonomous.
   */
  double speed = 0; 
  int desiredLeftPosition = 0;
  int desiredRightPosition = 0;
  int currentLeftPosition = 0;
  int currentRightPosition = 0;
  int initialRightPosition = 0;
  int initialLeftPosition= 0;
  double distToTravel_in = 0;
 
   
  DriveTrain roboDT;
  public AutonomousDrive(DriveTrain robotDT, double inchesToTravel) {
    // Use addRequirements() here to declare subsystem dependencies.
    roboDT = robotDT;
    distToTravel_in = inchesToTravel;
     // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboDT);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialLeftPosition = (int)roboDT.getLeftPosition();
    initialRightPosition = (int)roboDT.getRightPosition();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roboDT.arcadeDrive(0, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    roboDT.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    desiredLeftPosition = roboDT.inchesToCounts(distToTravel_in, 
    Constants.GenConstants.REV_ENCODER_CPR)+initialLeftPosition;
    currentLeftPosition = (int)roboDT.getLeftPosition();
    desiredRightPosition = roboDT.inchesToCounts(distToTravel_in, 
    Constants.GenConstants.REV_ENCODER_CPR)+initialRightPosition;
    currentRightPosition = (int)roboDT.getRightPosition();
    if ((currentRightPosition >= desiredRightPosition) && (currentLeftPosition>=desiredLeftPosition)){
      return true;
  } else {
    return false;
  }
   
  }
}
