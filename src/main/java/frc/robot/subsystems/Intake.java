package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    private SparkFlex intakeMotor;
    /*
     *  COUNTERCLOCKWISE IS POSITIVE
     *
     * Clockwise = Ball In / Tube Out
     * Counter Clockwise = Ball Out / Tube In
     */

    private double speed;
    
    public Intake(){
        intakeMotor = new SparkFlex(Constants.intakeMotorID, MotorType.kBrushless);
    
        speed = 0.5;
    }

    public void BallIn_TubeOut(){
        // Right Trigger RT
        intakeMotor.set(-speed);
    }

    public void BallOut_TubeIn(){
        // Left Trigger LT
        intakeMotor.set(speed);
    }
}