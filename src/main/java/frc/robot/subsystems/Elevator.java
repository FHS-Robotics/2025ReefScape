package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
public class Elevator extends SubsystemBase {

    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;

    private RelativeEncoder encoder; // Use one encoder for control

    // PID Controller (Tune these values)
    private PIDController pidController;

    private double setpoint; // Desired elevator position
    private double maxHeight = 105.6;
    private double minHeight = 0;

    private double manualSpeed;

    private boolean killSwitch;

    public Elevator() {
    
        elevatorLeft = new SparkFlex(Constants.elevatorLeftID, MotorType.kBrushless);
        elevatorRight = new SparkFlex(Constants.elevatorRightID, MotorType.kBrushless);

        encoder = elevatorRight.getEncoder();

        //kp controls speed
        pidController = new PIDController(0.1, 0.0, 0.0);
        
        pidController.setTolerance(0.1); // Small error tolerance

        manualSpeed = 0.6;

        setpoint = 0.0;

        killSwitch = false;     
    }

    public void toggleKillSwitch(){
        if(killSwitch == true){
            killSwitch = false;
        }
        else if(killSwitch == false){
            killSwitch = true;

            setpoint = encoder.getPosition();
        }
    }
    public void moveUp(){
        setpoint = setpoint + manualSpeed;
        
        if(setpoint > maxHeight){
            setpoint = maxHeight;
        }
        
    }

    public void moveDown(){
        setpoint = setpoint - manualSpeed;

        if(setpoint < minHeight){
            setpoint = minHeight;
        }
    }

    public void setHeight(double targetPosition) {
        
        setpoint = targetPosition;
        
        if(setpoint > maxHeight){
            setpoint = maxHeight;
        }
        
        if(setpoint < minHeight){
            setpoint = minHeight;
        }
    }

    public boolean isAtSetpoint() {
        return pidController.atSetpoint(); //may not be needed
    }    

    @Override // Runs every 10 ms
    public void periodic() {
        if(killSwitch == false){
            
            SmartDashboard.putNumber("Elevator Setpoint", setpoint);
            double position = encoder.getPosition(); 
            double speed = pidController.calculate(position, setpoint);

            // Apply the same speed to both motors for sync
            elevatorRight.set(speed);
            elevatorLeft.set(-speed);

            if(-0.3 > RobotContainer.getLeftYValue()){
                moveUp();
            }
            if(0.3 < RobotContainer.getLeftYValue()){
                moveDown();
            }
        }
        if(RobotContainer.rightBumperPressed()){
            toggleKillSwitch();
        }
    
    }
}