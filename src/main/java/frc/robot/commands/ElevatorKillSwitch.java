package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorKillSwitch extends Command{
    private Elevator elevator;
    
    public ElevatorKillSwitch(Elevator elevator){
        this.elevator = elevator;

        addRequirements(elevator);
    }
    
    @Override
    public void initialize(){
        elevator.toggleKillSwitch();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
