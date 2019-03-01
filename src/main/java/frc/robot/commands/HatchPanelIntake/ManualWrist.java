/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HatchPanelIntake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchPanelIntake;
import lib.frc1747.controller.Xbox;
import lib.frc1747.subsytems.HBRSubsystem;

public class ManualWrist extends Command {
  HatchPanelIntake hatch;
  boolean PIDEnabled;
  double input;
  public ManualWrist() {
    hatch = HatchPanelIntake.getInstance();
    requires(hatch);
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
    input = OI.getInstance().getOperator().getAxis(Xbox.LEFT_VERTICAL);
    	
    	if (Math.abs(input) < 0.01 && !PIDEnabled){
    		hatch.setMode(HatchPanelIntake.Follower.WRIST, HBRSubsystem.Mode.PID);
    		hatch.setSetpoint(HatchPanelIntake.Follower.WRIST, hatch.getWristPosition());
    		hatch.setEnabled(true);
    		PIDEnabled = true;
    	}else if(Math.abs(input) >= 0.01 && Elevator.getInstance().getDistance() >= 13){
    		PIDEnabled = false;
    		hatch.setEnabled(false);
    		hatch.setWristPower(-input * 0.5);
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
    hatch.setMode(HatchPanelIntake.Follower.WRIST, HBRSubsystem.Mode.PID);
    hatch.setSetpoint(HatchPanelIntake.Follower.WRIST, hatch.getWristPosition());
    hatch.setEnabled(true);
    hatch.setWristPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
