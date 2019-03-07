/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Elevator;

import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.CargoScoring;
import frc.robot.subsystems.Elevator;
import lib.frc1747.motion_profile.Parameters;
import lib.frc1747.subsytems.HBRSubsystem;

public class ElevatorProfiles extends Command {
  public Elevator.ElevatorPositions position;
  public static Elevator elevator;
  int stage;
  Elevator.ElevatorPositions pos;
  public ElevatorProfiles(Elevator.ElevatorPositions pos){
    this.pos = pos;
    stage = 1;
    elevator = Elevator.getInstance();
    requires(elevator);
    setInterruptible(false);
  }
  public ElevatorProfiles(int stage) {
    this.stage = stage;
    elevator = Elevator.getInstance();
    requires(elevator);
    setInterruptible(false);
    
  }
  

  // Called just before this Command runs the first time
  @Override
  protected void initialize() { 
    if(stage == 1){
      if(CargoScoring.getInstance().sensorActivated()){
        this.position = Elevator.ElevatorPositions.C1;
      }else{
        this.position = Elevator.ElevatorPositions.HP1;
      }
    } else if(stage == 2){
      if(CargoScoring.getInstance(
      ).sensorActivated()){
        this.position = Elevator.ElevatorPositions.C2;
      }else{
        this.position = Elevator.ElevatorPositions.HP2;
      }
    } else if(stage == 3){
      if(CargoScoring.getInstance().sensorActivated()){
        this.position = Elevator.ElevatorPositions.C3;
      }else{
        this.position = Elevator.ElevatorPositions.HP3;
      }
    }else{
      this.position = Elevator.ElevatorPositions.CSHIP;
    }

    if(pos != null){
      this.position = pos;
    }

    if(Math.abs(Elevator.positions[position.ordinal()] - elevator.getDistance()) >= 1){
      double[][][] profiles = HBRSubsystem.generateSkidSteerPseudoProfile(Elevator.positions[position.ordinal()] - elevator.getDistance(), 0, Parameters.I_SAMPLE_LENGTH * 12, 120, 200, 9000.1, Parameters.W_WIDTH, RobotMap.DT, true, true);
      
      for(int i = 0; i < profiles[0].length; i++){
        profiles[0][i][0] += elevator.getDistance();
      }

      elevator.setMode(Elevator.Follower.ELEVATOR, HBRSubsystem.Mode.FOLLOWER);
      elevator.setPIDMode(Elevator.Follower.ELEVATOR, HBRSubsystem.PIDMode.POSITION);
      elevator.setILimit(Elevator.Follower.ELEVATOR, 0);
      elevator.setOutputLimit(Elevator.Follower.ELEVATOR, 0.7);
      elevator.setFeedforward(Elevator.Follower.ELEVATOR, 0, GambeziDashboard.get_double("Elevator/kV"), GambeziDashboard.get_double("Elevator/kA"));
      elevator.setFeedback(Elevator.Follower.ELEVATOR, GambeziDashboard.get_double("Elevator/P"), GambeziDashboard.get_double("Elevator/I"), GambeziDashboard.get_double("Elevator/D"));
      elevator.resetIntegrator(Elevator.Follower.ELEVATOR);
      elevator.setProfile(Elevator.Follower.ELEVATOR, profiles[0]);
      elevator.resume(Elevator.Follower.ELEVATOR);
      elevator.setEnabled(true);
    }
    elevator.tellSetpoint(Elevator.positions[position.ordinal()]);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
  //  return !OI.getInstance().getDriver().getButton(Xbox.A).get();
      return (Math.abs(elevator.getDistance() - Elevator.positions[position.ordinal()])) < 1 || !elevator.isRunning(Elevator.Follower.ELEVATOR);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    elevator.setMode(Elevator.Follower.ELEVATOR, HBRSubsystem.Mode.PID);
    elevator.setSetpoint(Elevator.Follower.ELEVATOR, Elevator.positions[position.ordinal()]);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
