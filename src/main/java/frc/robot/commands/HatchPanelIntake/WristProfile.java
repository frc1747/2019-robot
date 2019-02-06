/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HatchPanelIntake;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.HatchPanelIntake;
import lib.frc1747.subsytems.HBRSubsystem;
import lib.frc1747.motion_profile.Parameters;

public class WristProfile extends Command {
  HatchPanelIntake hpIntake;
  HatchPanelIntake.Positions position;

  public WristProfile(HatchPanelIntake.Positions position) {
    hpIntake = HatchPanelIntake.getInstance();
    requires(hpIntake);
    this.position = position;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      hpIntake.setMode(HatchPanelIntake.Follower.WRIST, HBRSubsystem.Mode.PID);
      hpIntake.setPIDMode(HatchPanelIntake.Follower.WRIST, HBRSubsystem.PIDMode.POSITION);
      hpIntake.setILimit(HatchPanelIntake.Follower.WRIST, 0);
      hpIntake.setFeedforward(HatchPanelIntake.Follower.WRIST, 0.0, 0.0, 0.0);
      hpIntake.setFeedback(HatchPanelIntake.Follower.WRIST, 0.38, 0.0, 0.0);
      hpIntake.resetIntegrator(HatchPanelIntake.Follower.WRIST);
      hpIntake.setEnabled(true);
      hpIntake.setSetpoint(HatchPanelIntake.Follower.WRIST, HatchPanelIntake.positions[position.ordinal()]);
      hpIntake.tellSetpoint(HatchPanelIntake.positions[position.ordinal()]);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
     
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
