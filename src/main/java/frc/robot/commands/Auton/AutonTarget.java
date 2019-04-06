/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auton;

import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import lib.frc1747.subsytems.HBRSubsystem;

public class AutonTarget extends Command {
  Drivetrain drivetrain;
  long duration;
  long startTime;
  double tv;
  double tx;
  double tx0;
  double tx1;
  double ty;
  double ty0;
  double ty1;
  double ta;
  double ta0;
  double ta1;
  double thor;
  double tvert;
  double ax0;
  double ax1;
  double xDistscaled;
  double ay0;
  double ay1;
  double h1 = 33.25;
  double h2 = 29.25;
  double hFOV = 54;
  double vFOV = 41;
  double vpw = 2 * Math.tan((hFOV*Math.PI/180)/2);
  double vph = 2 * Math.tan((vFOV*Math.PI/180)/2);
  double x0;
  double y0;
  double x1;
  double y1;
  double avTheta;
  double d0;
  double d1;
  double o0;
  double o1;
  double phi;
  double j0;
  double j1;
  double avJ;
  double yDist;
  double xDist;
  double currentDistance;
  double currentAngle;
  double errorAngle;
  double leg;
  int stage = 1;
  boolean leftSide;
  boolean centered;
  boolean two_right;
  boolean two_top;
  double [] xCorners;
  double [] yCorners;

  public AutonTarget(long duration) {
    this.duration = duration;
    drivetrain = Drivetrain.getInstance();
    requires(drivetrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(1);
    startTime = System.currentTimeMillis();
    currentDistance = (drivetrain.getLeftDistance()+drivetrain.getRightDistance())/2;
    currentAngle = drivetrain.getAngle();
    currentDistance = (drivetrain.getLeftDistance()+drivetrain.getRightDistance())/2;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
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
    if(tv > 0) {
      stage = 1;
      xCorners = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcornx").getDoubleArray(new double[0]);
      yCorners = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcorny").getDoubleArray(new double[0]);

      tx0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx0").getDouble(0);
      tx1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx1").getDouble(0);
      if(tx0 < tx1){
        tx0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx0").getDouble(0);
        tx1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx1").getDouble(0);
        ta0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta0").getDouble(0);
        ta1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta1").getDouble(1);
        ty0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty0").getDouble(0);
        ty1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty1").getDouble(0);
      }else{
        tx0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx1").getDouble(0);
        tx1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx0").getDouble(0);
        ta0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta1").getDouble(0);
        ta1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta0").getDouble(1);
        ty0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty1").getDouble(0);
        ty1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty0").getDouble(0);
      }
      // tx0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx0").getDouble(0);
      // tx1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx1").getDouble(0);
      // ta0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta0").getDouble(0);
      // ta1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta1").getDouble(1);
      // ty0 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty0").getDouble(0);
      // ty1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty1").getDouble(0);
      ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

      GambeziDashboard.set_double("limelight/tx0", tx0);
      GambeziDashboard.set_double("limelight/tx1", tx1);
      GambeziDashboard.set_double("limelight/ty0", ty0);
      GambeziDashboard.set_double("limelight/ty1", ty1);

      x0 = vpw/(2) * tx0;
      y0 = vph/(2) * ty0;
      x1 = vpw/(2) * tx1;
      y1 = vph/(2) * ty1;

      ax0 = Math.atan2(x0, 1);
      ay0 = Math.atan2(y0, 1);
      ax1 = Math.atan2(x1, 1);
      ay1 = Math.atan2(y1, 1);

      GambeziDashboard.set_double("limelight/ax0", ax0);
      GambeziDashboard.set_double("limelight/ax1", ax1);
      GambeziDashboard.set_double("limelight/ay0", ay0);
      GambeziDashboard.set_double("limelight/ay1", ay1);
      avTheta = (ax0 + ax1)/2;
      GambeziDashboard.set_double("limelight/avTheta", avTheta);

      d0 = 0.5 * (h2 - h1) / Math.tan(ay0);
      d1 = 0.5 * (h2 - h1) / Math.tan(ay1);
      GambeziDashboard.set_double("limelight/d0", d0);
      GambeziDashboard.set_double("limelight/d1", d1);

      o0 = d0 * Math.tan(ax0);
      o1 = d1 * Math.tan(ax1);
      GambeziDashboard.set_double("limelight/o0", o0);
      GambeziDashboard.set_double("limelight/o1", o1);

      // phi = Math.acos((o1-o0)/10.5);
      // leg = Math.sqrt(10.5 * 10.5-(o1 - o0) * (o1 - o0));
      phi = Math.atan2(d1 - d0, o1 - o0)/2.75;
      GambeziDashboard.set_double("limelight/phi", phi);
      GambeziDashboard.set_double("limelight/newphi", Math.atan2(leg, o1 - o0));

      j0 = o0 * Math.tan(phi);
      j1 = o1 * Math.tan(phi);
      GambeziDashboard.set_double("limelight/tanphi", Math.tan(phi));

      GambeziDashboard.set_double("limelight/j0", j0);
      GambeziDashboard.set_double("limelight/j1", j1);

      // avJ = (j0+j1)/2;
      // if(Math.abs(ta0) > Math.abs(ta1)){
      //   leftSide = true;
      // }else{
      //   leftSide = false;
      // }
      try{
        GambeziDashboard.set_double("limelight/xCorner 2", xCorners[2]);
        GambeziDashboard.set_double("limelight/xCorner 5", xCorners[5]);
        GambeziDashboard.set_double("limelight/yCorner 2", yCorners[2]);
        GambeziDashboard.set_double("limelight/yCorner 5", yCorners[5]);
        if(xCorners[2] > xCorners[5]){
          GambeziDashboard.set_boolean("limelight/2 on right", true);
          two_right = true;
        }else{
          GambeziDashboard.set_boolean("limelight/2 on right", false);
          two_right = false;
        }
        if(yCorners[2] < yCorners[5]){
          GambeziDashboard.set_boolean("limelight/2 on top", true);
          two_top = true;
        }else{
          GambeziDashboard.set_boolean("limelight/2 on top", false);
          two_top = false;
        }
  
        if(two_right){
          if(two_top){
            leftSide = true;
          }else{
            leftSide = false;
          }
        }else{
          if(two_top){
            leftSide = false;
          }else{
            leftSide = true;
          }
        }
      }
      catch(Exception e) {
        if (ty0 > ty1){
          leftSide = true;
        } else {
          leftSide = false;
        }
      }
      GambeziDashboard.set_boolean("limelightleftSide", leftSide);

      if(leftSide){
        yDist = (d0 - j0) * Math.cos(phi);
      }else{
        yDist = (d1 - j1) * Math.cos(phi);
      }
      // yDist = (d0 - j0) * Math.cos(phi);
      xDist = 1.25 * yDist / Math.tan(Math.PI / 2 - phi - avTheta) ;

      GambeziDashboard.set_double("limelight/xDist", xDist);
      GambeziDashboard.set_double("limelight/yDist", yDist);
      drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, 4);
      drivetrain.setEnabled(true);
      GambeziDashboard.set_double("limelight/aRatio", ta0/ta1);
      if(0.95 < (ta0/ta1) && (ta0/ta1) < 1.05){
        centered = true;
      }else{
        centered = false;
      }
      if(xDist > 36){
        xDist = 36;
      }
      // if(xDist >= 0){
      //   leftSide = true;
      // }else{
      //   leftSide = false;
      // }

    }
    currentAngle = drivetrain.getAngle();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    GambeziDashboard.set_double("stage", stage);
    double angleSetpoint;
    if(leftSide){
      angleSetpoint = ax1;
      // angleSetpoint = Math.PI/4;

    }else{
      angleSetpoint = ax0;
      // angleSetpoint = -Math.PI/4;
    }
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    GambeziDashboard.set_double("Angle Error", drivetrain.getAngle() - (currentAngle - phi - ax1));
    if(stage == 1) {
      if(Math.abs(drivetrain.getAngle() - (currentAngle - phi - angleSetpoint)) > 0.05) {
        drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, -(drivetrain.getAngle() - (currentAngle - phi - angleSetpoint)) * 1 * RobotMap.A_V_MAX);
      }
      else {
        drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, 0);
        stage = 2;
      }
    }
    if(stage == 2){
      xDistscaled = Math.abs(xDist / Math.cos(Math.PI/2 - 2*phi - angleSetpoint));
      GambeziDashboard.set_double("limelight/xDistscaled", xDistscaled);
      if(Math.abs(xDistscaled) > 24){
        xDistscaled = 24;
      }
      if((drivetrain.getLeftDistance()+drivetrain.getRightDistance())/2 - currentDistance >=  Math.abs(xDistscaled/12)) {
        drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, 0);
        stage = 3;
      }
    }
    if(stage == 3) {
      if(Math.abs(drivetrain.getAngle() - (currentAngle + phi)) > 0.1) {
        drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, -(drivetrain.getAngle() - (currentAngle + phi)) * 1 * RobotMap.A_V_MAX);
      }
      else {
        drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, 0);
        stage = 4;
      }
    }
    if(stage == 4){
      double linearOutput = (46-ta) * 0.01 * RobotMap.S_V_MAX;
      if (linearOutput > 3){
        linearOutput = 3;
      }
      drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, linearOutput);
      drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, -tx * 0.02 * RobotMap.A_V_MAX);
    }

    // else if(hasRun){
    //   if(tv < 1.0){
    //     drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, 0);
    //     drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, 0);
    //   }else{
    //     double linearOutput = (31.2-ta) * 0.01 * RobotMap.S_V_MAX;
    //     if (linearOutput > 6){
    //       linearOutput = 6;
    //     }
    //     drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, linearOutput);
    //     drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, -tx * 0.04 * RobotMap.A_V_MAX);
    //     // drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, (1 - (ta0 / ta1)) * 0.2 * RobotMap.A_V_MAX * ((31.2 - ta) / 31.2) + -tx * 0.015 * RobotMap.A_V_MAX);
    //     GambeziDashboard.set_double("target distance setpoint", (31.2-ta) * 0.02 * RobotMap.S_V_MAX);
    //     GambeziDashboard.set_double("target angle setpoint", (1 - (ta0 / ta1)) * 0.2 * RobotMap.A_V_MAX * ((31.2 - ta) / 31.2) + -tx * 0.04 * RobotMap.A_V_MAX);
    //   }
    // }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return System.currentTimeMillis() - startTime >= duration;
    // return hasRun && drivetrain.getAngle() == 2*phi + ax1;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(0);
    
    // drivetrain.setSetpoint(Drivetrain.Follower.DISTANCE, 0);
    // drivetrain.setSetpoint(Drivetrain.Follower.ANGLE, drivetrain.getAngle());
    // drivetrain.setEnabled(false);
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
