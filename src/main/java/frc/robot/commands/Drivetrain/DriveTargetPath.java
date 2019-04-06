/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import lib.frc1747.motion_profile.Parameters;
import lib.frc1747.subsytems.HBRSubsystem;

public class DriveTargetPath extends Command {
  Drivetrain drivetrain;
  public DriveTargetPath() {
    drivetrain = Drivetrain.getInstance();
    requires (drivetrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
    if(tv >= 0.0){
      // System.out.println("drive Target path========================================");
      double[][][] profiles = HBRSubsystem.generateSkidSteerPseudoProfile(66.0402824578 * Math.exp(-.2492202696 * ta) / 12, -tx * Math.PI / 180, Parameters.I_SAMPLE_LENGTH, 14, 20, 26, Parameters.W_WIDTH, RobotMap.DT, true, true);
    	double linearOffset = (drivetrain.getLeftDistance() + drivetrain.getRightDistance()) / 2;
    	double angularOffset = drivetrain.getAngle();
    	for(int i = 0;i < profiles[0].length;i++) {
    		profiles[0][i][0] += linearOffset;
    		profiles[1][i][0] += angularOffset;
      }
    
        // distance PID
      drivetrain.setMode(Drivetrain.Follower.DISTANCE, HBRSubsystem.Mode.FOLLOWER);
      drivetrain.setPIDMode(Drivetrain.Follower.DISTANCE, HBRSubsystem.PIDMode.POSITION);
      drivetrain.setILimit(Drivetrain.Follower.DISTANCE, 0);
      drivetrain.setFeedforward(Drivetrain.Follower.DISTANCE, 0, GambeziDashboard.get_double("Drivetrain/Distance kV"), GambeziDashboard.get_double("Drivetrain/Distance kA"));
      drivetrain.setFeedback(Drivetrain.Follower.DISTANCE, GambeziDashboard.get_double("Drivetrain/Distance P"), 0, GambeziDashboard.get_double("Drivetrain/Distance D"));
      drivetrain.resetIntegrator(Drivetrain.Follower.DISTANCE);

      // angle PID
      drivetrain.setMode(Drivetrain.Follower.ANGLE, HBRSubsystem.Mode.FOLLOWER);
      drivetrain.setPIDMode(Drivetrain.Follower.ANGLE, HBRSubsystem.PIDMode.POSITION);
      drivetrain.setILimit(Drivetrain.Follower.ANGLE, 0);
      drivetrain.setFeedforward(Drivetrain.Follower.ANGLE, 0, GambeziDashboard.get_double("Drivetrain/Angle kV"), GambeziDashboard.get_double("Drivetrain/Angle kA"));
      drivetrain.setFeedback(Drivetrain.Follower.ANGLE, GambeziDashboard.get_double("Drivetrain/Angle P"), 0, GambeziDashboard.get_double("Drivetrain/Angle D"));
      drivetrain.resetIntegrator(Drivetrain.Follower.ANGLE);
      
      drivetrain.setProfile(Drivetrain.Follower.DISTANCE,profiles[0]);
      drivetrain.setProfile(Drivetrain.Follower.ANGLE,profiles[1]);
    
      drivetrain.resume(Drivetrain.Follower.DISTANCE);
      drivetrain.resume(Drivetrain.Follower.ANGLE);
      drivetrain.setEnabled(true);

      
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !drivetrain.isRunning(Drivetrain.Follower.DISTANCE);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drivetrain.setEnabled(false);
    drivetrain.setLeftPower(0);
    drivetrain.setRightPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
