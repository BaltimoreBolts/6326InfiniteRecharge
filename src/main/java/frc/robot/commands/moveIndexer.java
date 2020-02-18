/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class moveIndexer extends CommandBase {
  Indexer roboIndexer;
  double speed = 0; 
  int currentPosition = 0;
  int desiredPosition = 0;
  int initialPosition= 0;
  /**
   * Creates a new moveIndexer.
   */
  public moveIndexer(Indexer roboIndexer) {
    roboIndexer = roboIndexer;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(roboIndexer);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = roboIndexer.getdesiredSpeed();
    initialPosition = roboIndexer.getEncoderValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roboIndexer.Movement(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //The speed is being set to 0/
    roboIndexer.Movement(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   desiredPosition = roboIndexer.degreeToCounts(120, 8192)+initialPosition;
    currentPosition = roboIndexer.getEncoderValue();
    if (currentPosition >= desiredPosition){
      return true;
  } else{
    return false;
  }
    
  }
}
