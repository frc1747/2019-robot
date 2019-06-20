/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import lib.frc1747.controller.Xbox;
import lib.frc1747.subsytems.HBRSubsystem;

public class DriveWithJoysticks extends Command {
  Drivetrain drivetrain;
  double leftVert;
  double rightHoriz;
  double theoreticalMax;
  // boolean state = false;
  // boolean laststate = false;
  boolean shouldstop = true;

  public DriveWithJoysticks() {
    drivetrain = Drivetrain.getInstance();
    requires (drivetrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // distance PID
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
  //  drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, drivetrain.getAngle());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      //   if (0.1 > Math.abs(OI.getInstance().getDriver().getAxis(Xbox.RIGHT_HORIZONTAL)) && !shouldstop){
      //   shouldstop = true;
      //   drivetrain.setSetpoint(DriveSub.Follower.ANGLE, drivetrain.getAngle());
      //   drivetrain.setLeftPower(0);
      //   drivetrain.setRightPower(0);
      // }
      
      // if (shouldstop && 0.1 <= Math.abs(OI.getInstance().getDriver().getAxis(Xbox.RIGHT_HORIZONTAL))){
      //   shouldstop = false;
      // }

      // else if (!shouldstop) {
      // drivetrain.setSetpoint(DriveSub.Follower.ANGLE, drivetrain.getAngle() + OI.getInstance().getDriver().getAxis(Xbox.RIGHT_HORIZONTAL) * Math.PI);
      // }

      // if (OI.getInstance().getDriver().getLTButton().get()){
      //   drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, leftVert * leftVert * leftVert * RobotMap.S_V_MAX / 5);
      //   drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, rightHoriz * rightHoriz * rightHoriz * RobotMap.A_V_MAX / 5);
      // } else {

      //   drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, leftVert * leftVert * leftVert * RobotMap.S_V_MAX * (77.5 - (Elevator.getInstance().getDistance() / 2) / 77.5));
      //   drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, rightHoriz * rightHoriz * rightHoriz * RobotMap.A_V_MAX * (77.5 - (Elevator.getInstance().getDistance() / 2)) / 77.5);
      // }

      leftVert = OI.getInstance().getDriver().getAxis(Xbox.LEFT_VERTICAL) ;
      rightHoriz = -OI.getInstance().getDriver().getAxis(Xbox.RIGHT_HORIZONTAL) ;
    if (OI.getInstance().getOperator().getRTButton().get()){
      theoreticalMax = RobotMap.ACTUAL_S_V_MAX;
    } else {
      theoreticalMax = RobotMap.S_V_MAX;
    }
    if(Elevator.getInstance().getDistance() > 30){
      
        drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, OI.getInstance().getDriver().getAxis(Xbox.LEFT_VERTICAL) * RobotMap.S_V_MAX/5);
        drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, -OI.getInstance().getDriver().getAxis(Xbox.RIGHT_HORIZONTAL) * RobotMap.A_V_MAX/5);
        // drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, (OI.getInstance().getDriver().getAxis(Xbox.LEFT_VERTICAL) * RobotMap.S_V_MAX)/(Elevator.getInstance().getElevatorPosition()/(83.5/5)));
        // drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, (-OI.getInstance().getDriver().getAxis(Xbox.RIGHT_HORIZONTAL) * RobotMap.A_V_MAX)/(Elevator.getInstance().getElevatorPosition()/(83.5/5)));
    }else{
      if (OI.getInstance().getDriver().getLTButton().get()){
        drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, leftVert * leftVert * leftVert * theoreticalMax / 5);
        drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, rightHoriz * rightHoriz * rightHoriz * RobotMap.A_V_MAX / 5);
      } else {
        if (OI.getInstance().getDriver().getRTButton().get()){
          drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, leftVert * leftVert * leftVert * theoreticalMax);
          drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, rightHoriz * rightHoriz * rightHoriz * RobotMap.A_V_MAX / 3 * 2);
  
        }else{
         drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, leftVert * leftVert * leftVert * theoreticalMax);
          drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, rightHoriz * rightHoriz * rightHoriz * RobotMap.A_V_MAX);
        }
      }

      
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
    drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, 0);
    drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, drivetrain.getAngle());
    drivetrain.setEnabled(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
