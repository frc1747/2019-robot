/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Elevator;
import lib.frc1747.controller.Xbox;
import lib.frc1747.subsytems.HBRSubsystem;

public class ManualElevator extends Command {
  Elevator elev;
  boolean PIDEnabled;
  double input;
  public ManualElevator() {
    elev = Elevator.getInstance();
    requires(elev);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    PIDEnabled = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    input = OI.getInstance().getOperator().getAxis(Xbox.RIGHT_VERTICAL);
    	
    	if (Math.abs(input) < 0.01 && !PIDEnabled){
    		elev.setMode(Elevator.Follower.ELEVATOR, HBRSubsystem.Mode.PID);
        elev.setSetpoint(Elevator.Follower.ELEVATOR, elev.getDistance());
        elev.tellSetpoint(elev.getDistance());
    		elev.setEnabled(true);
    		PIDEnabled = true;
    	}else if(Math.abs(input) >= 0.01){
    		PIDEnabled = false;
    		elev.setEnabled(false);
    		elev.setPower(input * 0.5 + 0.3);
    	}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elev.setMode(Elevator.Follower.ELEVATOR, HBRSubsystem.Mode.PID);
    elev.setSetpoint(Elevator.Follower.ELEVATOR, elev.getDistance());
    elev.setEnabled(true);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
