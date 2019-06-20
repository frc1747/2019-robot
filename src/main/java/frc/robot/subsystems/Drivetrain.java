/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.tigerhuang.gambezi.dashboard.GambeziDashboard;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.commands.setLeftPIDPower;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Drivetrain.Drive;
import frc.robot.commands.Drivetrain.DriveWithJoysticks;
import lib.frc1747.subsytems.HBRSubsystem;

/**
 * Add your docs here.
 */
public class Drivetrain extends HBRSubsystem<Drivetrain.Follower> {
  static Drivetrain drivetrain;
  DriveSide left, right;
  private AHRS gyro;

  public Drivetrain() {
    super(RobotMap.DT);
    left = new DriveSide(RobotMap.DRIVE_LEFT_MOTOR_PORTS, RobotMap.DRIVE_MOTOR_INVERT_LEFT, 0, 1);
    right = new DriveSide(RobotMap.DRIVE_RIGHT_MOTOR_PORTS, RobotMap.DRIVE_MOTOR_INVERT_RIGHT, 3, 2);
    // gyro = new AHRS(SPI.Port.);
    gyro = new AHRS(SerialPort.Port.kUSB);
  }

  public enum Follower {
    DISTANCE, ANGLE;
  }

  public void setLeftPower(double power) {
    left.setPower(power);
  }

  public DriveSide getLeftSide() {
    return left;
  }

  public DriveSide getRightSide() {
    return right;
  }

  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public void setRightPower(double power) {
    right.setPower(power);
  }

  public double getAngle() {
    return Math.PI * -gyro.getAngle() / 180;
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getAngularVelocity() {
    return -gyro.getRate();
  }

  public double getLeftDistance() {
    return left.getDistance()/ RobotMap.LEFT_ENCODER_SCALING;
    // return left.getSpeed();
  }

  public double getRightDistance() {
    return right.getDistance() / RobotMap.RIGHT_ENCODER_SCALING;
    // return right.getSpeed();
  }

  public double getLeftSpeed() {
    return left.getSpeed() / RobotMap.LEFT_ENCODER_SCALING;
  }

  public double getRightSpeed() {
    return right.getSpeed() / RobotMap.RIGHT_ENCODER_SCALING;
  }
  
  public double getRightCurrent(){
    return right.getCurrent();
  }

  public double getLeftCurrent(){
    return left.getCurrent();
  }

  public double averageSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2;
  }

  public double averageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public static Drivetrain getInstance() {
    if (drivetrain == null) {
      drivetrain = new Drivetrain();
    }
    return drivetrain;
  }

  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoysticks());
    // setDefaultCommand(new Drive());
  }

  public void pidWrite(double[] power) {
    arcadeDrive(power[0], -power[1]);
    GambeziDashboard.set_double("Drivetrain/Power 0", power[0]);
    GambeziDashboard.set_double("Drivetrain/Power 1", power[1]);

    // SmartDashboard.putNumber("distance setpoint", power[0]);
    // SmartDashboard.putNumber("angle setpoint", power[1]);
  }

  public void errorsWrite(double[] errors) {

  }

  public double[][] pidRead() {
    double[][] input = new double[2][2];
    input[0][0] = averageDistance();
    input[1][0] = averageSpeed();
    input[0][1] = getAngle();
    input[1][1] = getAngularVelocity();
    return input;
  }

  public void internalVariablesWrite(double[] vars) {
    SmartDashboard.putNumber("Linear Velocity Setpoint", vars[0]);
    SmartDashboard.putNumber("Actual Linear Velocity", vars[1]);
    SmartDashboard.putNumber("Angular Velocity Setpoint", vars[2]);
    SmartDashboard.putNumber("Actual Angular Velocity", vars[3]);
    GambeziDashboard.set_double("Drivetrain/Linear Velocity Setpoint", vars[0]);
    GambeziDashboard.set_double("Drivetrain/Actual Linear Velocity", vars[1]);
    GambeziDashboard.set_double("Drivetrain/Angular Velocity Setpoint", vars[2]);
    GambeziDashboard.set_double("Drivetrain/Actual Angular Velocity", vars[3]);
  }

  public void arcadeDrive(double leftVert, double rightHoriz) {
    setLeftPower(leftVert + rightHoriz);
    setRightPower(leftVert - rightHoriz);
    GambeziDashboard.set_double("Drivetrain/Left Drive Power", leftVert + rightHoriz);
    GambeziDashboard.set_double("Drivetrain/Right Drive Power", leftVert - rightHoriz);

    SmartDashboard.putNumber("left power", leftVert + rightHoriz);
    SmartDashboard.putNumber("right power", leftVert - rightHoriz);
  }

  public class DriveSide {
    VictorSPX victor;
    VictorSPX victor2;
    TalonSRX talon;
    TalonSRX talon2;
    Encoder encoder;

    public DriveSide(int[] motorPorts, boolean[] motorInverted, int encport, int encport2) {
      talon = new TalonSRX(motorPorts[0]);
      talon.setInverted(motorInverted[0]);
      talon2 = new TalonSRX(motorPorts[1]);
      talon2.setInverted(motorInverted[1]);
      victor = new VictorSPX(motorPorts[2]);
      victor.setInverted(motorInverted[2]);
      victor2 = new VictorSPX(motorPorts[3]);
      victor2.setInverted(motorInverted[3]);
      encoder = new Encoder(encport, encport2);

    }

    public void PowerSetMotorGroup(int group, double power){
      victor.set(ControlMode.PercentOutput, power);
      // if (group == 0){
      //   victor.set(ControlMode.PercentOutput, power);
      // } else if (group == 1){
      //   victor2.set(ControlMode.PercentOutput, power);
      // } else if (group == 2){
      //   talon.set(ControlMode.PercentOutput, power);
      // } else if (group == 3){
      //   talon2.set(ControlMode.PercentOutput, power);
      // }
    }

    public double getCurrent(){
      return talon.getOutputCurrent() + talon2.getOutputCurrent();
    }

    public double getSpeed() {
      return encoder.getRate();
    }

    public double readValue() {
      return talon.getMotorOutputVoltage();
    }

    public void resetEncoder() {
      encoder.reset();
    }

    public void setPower(double power) {
      victor.set(ControlMode.PercentOutput, power);
      victor2.set(ControlMode.PercentOutput, power);
      talon.set(ControlMode.PercentOutput, power);
      talon2.set(ControlMode.PercentOutput, power);
    }

    public double getDistance() {
      return encoder.get();
      // return 0.0;
    }
  }

}
