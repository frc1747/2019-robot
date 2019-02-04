package lib.frc1747.subsytems;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import lib.frc1747.motion_profile.Parameters;

public class DriveCurveProfileGenerator {
	
//	private static DriveCurveProfileGenerator instance;
	
	private DriveCurveProfileGenerator(){
		
	}
	
	//make independant of RobotMap dt?
	public static void SaveDriveCurveProfile(double distance, double angle, String path, PathGenCfg params){
		double[][][] profiles = HBRSubsystem.generateSkidSteerPseudoProfile(distance, angle, Parameters.I_SAMPLE_LENGTH,
				params.v_max, params.a_max, params.j_max,
				Parameters.W_WIDTH, RobotMap.DT, true, true);
    	
		saveInternalProfile(profiles, path);
	}
	
	public static void saveInternalProfile(double[][][] profile, String path){
		try{
			PrintWriter pW = new PrintWriter(
					new BufferedWriter(
							new FileWriter(path)));
			
			pW.write(Double.toString(profile.length) + ", " + Integer.toString(profile[0].length) + "\n");
			
			for(int i=0;i < profile[0].length;i++){
				String line = "";
				
				for(int j=0;j < profile.length;j++){
					for(int k=0;k < profile[0][0].length;k++){
						line += Double.toString(profile[j][i][k]);
					}
				}
				
				pW.write(line + "\n");
			}
			
			pW.close();
			
		} catch(IOException e){
			e.printStackTrace();
		}
	}
	
	/*public static DriveCurveProfileGenerator getInstance(){
		if(instance == null){
			instance = new DriveCurveProfileGenerator();
		}
		return instance;
	}*/
}
