/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber;
import lib.frc1747.motion_profile.Parameters;
import lib.frc1747.subsytems.HBRSubsystem;

public class ClimberProfiles extends Command {

  public Climber.LiftHeights position;
  public static Climber climber;
  int stage;
  Climber.LiftHeights pos;
  
  public ClimberProfiles(Climber.LiftHeights posit){
    this.pos = posit;
    stage = 1;
    climber = Climber.getInstance();
    requires(climber);
    setInterruptible(false);
  }

  public ClimberProfiles(int stage) {
    this.stage = stage;
    climber = Climber.getInstance();
    requires(climber);
    setInterruptible(false);
  }

  @Override
  protected void initialize() {
    if(pos != null){
      this.position = pos;
    }
    this.position = pos;

    if(Math.abs(Climber.positions[position.ordinal()] - climber.getCurrentHeight()) >= 1){
      double[][][] profiles = HBRSubsystem.generateSkidSteerPseudoProfile(Climber.positions[position.ordinal()] - climber.getCurrentHeight(), 0, Parameters.I_SAMPLE_LENGTH * 12, 120, 200, 9000.1, Parameters.W_WIDTH, RobotMap.DT, true, true);
      
      for(int i = 0; i < profiles[0].length; i++){
        profiles[0][i][0] += climber.getCurrentHeight();
      }

      climber.setMode(Climber.Follower.LIFT, HBRSubsystem.Mode.FOLLOWER);
      climber.setPIDMode(Climber.Follower.LIFT, HBRSubsystem.PIDMode.POSITION);
      climber.setILimit(Climber.Follower.LIFT, 0);
      climber.setOutputLimit(Climber.Follower.LIFT, 0.775);
      climber.setFeedforward(Climber.Follower.LIFT, 0, GambeziDashboard.get_double("Climber/kV"), GambeziDashboard.get_double("Climber/kA"));
      climber.setFeedback(Climber.Follower.LIFT, GambeziDashboard.get_double("Climber/P"), GambeziDashboard.get_double("Climber/I"), GambeziDashboard.get_double("Climber/D"));
      climber.resetIntegrator(Climber.Follower.LIFT);
      climber.setProfile(Climber.Follower.LIFT, profiles[0]);
      climber.resume(Climber.Follower.LIFT);
      climber.setEnabled(true);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
  //  return !OI.getInstance().getDriver().getButton(Xbox.A).get();
      return (Math.abs(climber.getCurrentHeight() - Climber.positions[position.ordinal()])) < 1 || !climber.isRunning(Climber.Follower.LIFT);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber.setMode(Climber.Follower.LIFT, HBRSubsystem.Mode.PID);
    climber.setSetpoint(Climber.Follower.LIFT, Climber.positions[position.ordinal()]);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
