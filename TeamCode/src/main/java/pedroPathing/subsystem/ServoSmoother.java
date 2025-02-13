package pedroPathing.subsystem;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
ServoSmoother takes in the servo in a constructor. You then set your desired position and the servo
will slowly move to the position by doing incremtal position changes until it reaches the desired
position. Note that each ServoSmoother should call the update function every loop, otherwise the
position will not be updated.
 */
@Config
public class ServoSmoother {
    private Servo myServo = null;
    private Telemetry telemetry = null;
    private double targetPos = 0.0;
    private double currentPos = 0.0;
    public static double servoStep = 0.005;
    /*
    ServoSmoother takes in a servo to slow down.
     */
    public ServoSmoother(Servo toFix, Telemetry tele){
        myServo = toFix;
        telemetry = tele;
    }
    /*
    Set the desired position that we want to move to slowly.
     */
    public void setPosition(double position){
        if(position<0.0){
            targetPos = 0.0;
        } else if (position>1.0) {
            targetPos = 1.0;
        }
        else{
            targetPos = position;
        }
    }
    /*
    Move the servo 1 step closer to the target position or if it is at the target position do
    nothing. At startup this will do nothing until a first set position is set.
     */
    public void update() {
        telemetry.addData("servoPosition",currentPos);
        telemetry.addData("targetPosition",targetPos);
        if (currentPos == targetPos) {
            return;
        }
        if (currentPos < targetPos) {
            if (targetPos - currentPos <= servoStep) {
                currentPos = targetPos;
            } else {
                currentPos += servoStep;
            }
        } else {
            if (currentPos - targetPos <= servoStep) {
                currentPos = targetPos;
            } else {
                currentPos -= servoStep;
            }
        }
        myServo.setPosition(currentPos);
    }
    public void setPositionSlam(double position){
        if(position<0.0){
            targetPos = 0.0;
        } else if (position>1.0) {
            targetPos = 1.0;
        }
        else{
            targetPos = position;
        }
        currentPos=targetPos;
        myServo.setPosition(targetPos);
    }
}
