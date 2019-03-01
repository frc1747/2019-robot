// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.HatchPanelIntake;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.OI;
// import frc.robot.RobotMap;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.HatchPanelIntake;
// import lib.frc1747.subsytems.HBRSubsystem;
// import lib.frc1747.motion_profile.Parameters;

// public class GroundIntake extends Command {
//   HatchPanelIntake hatch;
//   Elevator elevator;
//   public GroundIntake() {
//     elevator = Elevator.getInstance();
//     hatch = HatchPanelIntake.getInstance();
//     requires(hatch);
//     requires(elevator);
//     // Use requires() here to declare subsystem dependencies
//     // eg. requires(chassis);
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {
//     elevator.setMode(Elevator.Follower.ELEVATOR, HBRSubsystem.Mode.FOLLOWER);
//     elevator.setPIDMode(Elevator.Follower.ELEVATOR, HBRSubsystem.PIDMode.POSITION);
//     if(Math.abs(Elevator.positions[Elevator.ElevatorPositions.C1.ordinal()] - elevator.getDistance()) >= 1){
//       double[][][] profiles = HBRSubsystem.generateSkidSteerPseudoProfile(Elevator.positions[Elevator.ElevatorPositions.C1.ordinal()] - elevator.getDistance(), 0, Parameters.I_SAMPLE_LENGTH * 12, 120, 200, 9000.1, Parameters.W_WIDTH, RobotMap.DT, true, true);
      
//       for(int i = 0; i < profiles[0].length; i++){
//         profiles[0][i][0] += elevator.getDistance();
//       }

//       elevator.resetIntegrator(Elevator.Follower.ELEVATOR);
//       elevator.setProfile(Elevator.Follower.ELEVATOR, profiles[0]);
//       elevator.resume(Elevator.Follower.ELEVATOR);
//       elevator.setEnabled(true);
//     }
// }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   protected void execute() {
//     if(Elevator.getInstance().getDistance() >= 14){
//       hatch.setRollerPower(0.5);
//       hatch.setSetpoint(HatchPanelIntake.Follower.WRIST, HatchPanelIntake.positions[HatchPanelIntake.Positions.GROUND.ordinal()]);  
//     }
    
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   protected boolean isFinished() {
//     return !OI.getInstance().getDriver().getLTButton().get();
//     // return false;
//   }

//   // Called once after isFinished returns true
//   @Override
//   protected void end() {
//     elevator.setMode(Elevator.Follower.ELEVATOR, HBRSubsystem.Mode.PID);
//     elevator.setSetpoint(Elevator.Follower.ELEVATOR, Elevator.positions[position.ordinal()]);
//     hatch.setRollerPower(0);
//     if(Math.abs(Elevator.positions[Elevator.ElevatorPositions.HP1.ordinal()] - elevator.getDistance()) >= 1){
//       double[][][] profiles = HBRSubsystem.generateSkidSteerPseudoProfile(Elevator.positions[Elevator.ElevatorPositions.HP1.ordinal()] - elevator.getDistance(), 0, Parameters.I_SAMPLE_LENGTH * 12, 120, 200, 9000.1, Parameters.W_WIDTH, RobotMap.DT, true, true);
      
//       for(int i = 0; i < profiles[0].length; i++){
//         profiles[0][i][0] += elevator.getDistance();
//       }

//       elevator.resetIntegrator(Elevator.Follower.ELEVATOR);
//       elevator.setProfile(Elevator.Follower.ELEVATOR, profiles[0]);
//       elevator.resume(Elevator.Follower.ELEVATOR);
//       elevator.setEnabled(true);
//     }
//   }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   protected void interrupted() {
//     end();
//     System.out.println("GroundIntake interrupted");
//   }
// }
