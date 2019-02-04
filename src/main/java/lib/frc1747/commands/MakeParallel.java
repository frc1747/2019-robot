package lib.frc1747.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class MakeParallel extends CommandGroup {

    public MakeParallel(Command ... commands) {
    	for(int i = 0;i < commands.length;i++) {
    		addParallel(commands[i]);
    	}
    }
}
