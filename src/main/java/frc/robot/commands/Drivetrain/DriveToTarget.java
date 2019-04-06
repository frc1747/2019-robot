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
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import lib.frc1747.controller.Xbox;
import lib.frc1747.subsytems.HBRSubsystem;

public class DriveToTarget extends Command {
  Drivetrain drivetrain;
  double tv;
  double tx;
  double ty;
  double ta;
  double ta0;
  double ta1;
  double thor;
  double tvert;

  
  public DriveToTarget() {
    drivetrain = Drivetrain.getInstance();
    requires(drivetrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    drivetrain.setMode(Drivetrain.Follower.DISTANCE, HBRSubsystem.Mode.PID);
    drivetrain.setPIDMode(Drivetrain.Follower.DISTANCE, HBRSubsystem.PIDMode.VELOCITY);
    drivetrain.setILimit(Drivetrain.Follower.DISTANCE, 0);
    drivetrain.setFeedforward(Drivetrain.Follower.DISTANCE, 0, GambeziDashboard.get_double("Drivetrain/teleop Distance kV"), 0);//0,1/S_V_MAX,0
    drivetrain.setFeedback(Drivetrain.Follower.DISTANCE, GambeziDashboard.get_double("Drivetrain/teleop Distance P"), 0, GambeziDashboard.get_double("Drivetrain/teleop Distance D")); // -0.005,0,0
    drivetrain.resetIntegrator(Drivetrain.Follower.DISTANCE);

    // angle PID
    drivetrain.setMode(Drivetrain.Follower.ANGLE, HBRSubsystem.Mode.PID);
    drivetrain.setPIDMode(Drivetrain.Follower.ANGLE, HBRSubsystem.PIDMode.VELOCITY);
    drivetrain.setILimit(Drivetrain.Follower.ANGLE, 0);
    drivetrain.setFeedforward(Drivetrain.Follower.ANGLE, 0, GambeziDashboard.get_double("Drivetrain/teleop Angle kV"), 0);// 0,1/(A_V_MAX * 4),0
    drivetrain.setFeedback(Drivetrain.Follower.ANGLE, GambeziDashboard.get_double("Drivetrain/teleop Angle P"), 0, GambeziDashboard.get_double("Drivetrain/teleop Angle D")); // 0.14,0,0
    drivetrain.resetIntegrator(Drivetrain.Follower.ANGLE);
    drivetrain.setEnabled(true);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    ta0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta0").getDouble(0);
    ta1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta1").getDouble(1);
    thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);


    // GambeziDashboard.set_double("box width", thor);
    // GambeziDashboard.set_double("box height", tvert);
    // GambeziDashboard.set_double("w/h", thor/tvert);


    if(tv < 1.0){
      drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, 0);
      drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, 0);
    }else{
      double linearOutput = (31.2-ta) * 0.01 * RobotMap.S_V_MAX;
      if (linearOutput > 6){
        linearOutput = 6;
      }
      drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, linearOutput);
      drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, -tx * 0.02 * RobotMap.A_V_MAX);
      // drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, (1 - (ta0 / ta1)) * 0.2 * RobotMap.A_V_MAX * ((31.2 - ta) / 31.2) + -tx * 0.015 * RobotMap.A_V_MAX);
      // GambeziDashboard.set_double("target distance setpoint", (31.2-ta) * 0.01 * RobotMap.S_V_MAX);
      // GambeziDashboard.set_double("target angle setpoint", (1 - (ta0 / ta1)) * 0.2 * RobotMap.A_V_MAX * ((31.2 - ta) / 31.2) + -tx * 0.04 * RobotMap.A_V_MAX);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !OI.getInstance().getDriver().getButton(Xbox.B).get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, 0);
    drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, drivetrain.getAngle());
    drivetrain.setEnabled(false);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
