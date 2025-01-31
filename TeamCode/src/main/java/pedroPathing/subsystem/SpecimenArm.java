package pedroPathing.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SpecimenArm {

    HardwareMap hardwareMap = null;
    Telemetry telemetry = null;
    public enum ShoulderState {COLLECT, ENTER, STOW, SCORE}
    public enum ClawState {OPEN, CLOSE}
    private ClawState clawState = ClawState.OPEN;
    private Servo leftShoulder = null;
    private Servo claw = null;
    public DcMotor rightShoulder = null;
    private ServoImplEx leftShoulderEx = null;
    private ShoulderState shoulderPosition = ShoulderState.COLLECT;
    private ElapsedTime specimenArmTime = new ElapsedTime();
    private ElapsedTime specimenClawTime = new ElapsedTime();
    private ElapsedTime latchTime = new ElapsedTime();
    private boolean specimenArmDone = true;
    private boolean specimenClawDone = true;
    private boolean scoreRequested = false;
    private double shoulderTimerThreshold = 500.0;
    public static int R_SHOULDER_STOW_POS = 0; //0.07;
    public static int R_SHOULDER_COLLECT_POS = 485; //500;//495;//490;//500; //440; //125;
    public static int R_SHOULDER_ENTER_POS = 120;//113;//125; //490;
    public static int R_SHOULDER_SCORE_POS = 219;//260; //235;
    public static double CLAW_OPEN = 1.00; //0.95; //0.85;
    public static double CLAW_CLOSE = 0.61; //0.25;
    public static double L_SHOULDER_STOW_POS = 0.12; //0.07;
    public static double L_SHOULDER_COLLECT_POS = 0.04;//0.05;
    public static double L_SHOULDER_ENTER_POS = 0.6;//0.55;
    public static double R_SHOULDER_INTAKE_POS = 0.67;  //11 inches even
    public static double R_SHOULDER_PLACE_POS = 0.48;//26.75 inches
    public static double R_SHOULDER_PRE_SCORE_POS = 0.35; //New variable for Idaho
    public static double SHOULDER_OFFSET = 0.021;
    public static double SPECIMENARM_MOVE_TIME = 1000.0;
    public static double SPECIMENCLAW_OPEN_TIME = 1000.0;
    public static double LATCH_TIME_THRESHOLD = 150.0;
    public static double SCORE_LATCH_TIME = 1.0;
    public static double LATCH_SPEED = 0.8;
    public static double MOVE_SPEED = 0.4;
    public static double COLLECT_SPEED =0.0;
    public static double R_SHOULDER_RESET_POWER = -0.2;



    public SpecimenArm(HardwareMap hm, Telemetry tele) {
        hardwareMap = hm;
        telemetry = tele;
        telemetry.addData("Specimen Arm Status", "initializing");


        leftShoulder = hardwareMap.get(Servo.class, "left_shoulder"); // Intake: 11 even
        claw = hardwareMap.get(Servo.class, "claw");
        rightShoulder = hardwareMap.get(DcMotor.class, "right_shoulder"); // Ready: 26.75
        leftShoulderEx = hardwareMap.get(ServoImplEx.class, "left_shoulder");
        rightShoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(){
        if(scoreRequested && latchTime.milliseconds()>LATCH_TIME_THRESHOLD){
            scoreRequested = false;
            //rightShoulder.setTargetPosition(R_SHOULDER_SCORE_POS);
            //rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightShoulder.setPower(0);
        }
    }
    public void nextClawState() {
        if (clawState == ClawState.OPEN) {
            clawStateClose();
        }
        else {
            clawStateOpen();
            //shoulderPosition = ShoulderState.COLLECT;
            goToCollectArmState();
        }
    }
    public void clawStateOpen(){
        claw.setPosition(CLAW_OPEN);
        clawState = ClawState.OPEN;
    }
    public void clawStateClose(){
        claw.setPosition(CLAW_CLOSE);
        clawState = ClawState.CLOSE;
    }
    /*
    public class doAutoClawStateOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (specimenClawDone) {
                specimenClawDone = false;
                specimenClawTime.reset();
                clawStateOpen();
                return true;
            }
            if (specimenClawTime.milliseconds() < SPECIMENCLAW_OPEN_TIME) {
                return true;
            }
            specimenClawDone = true;
            return false;
        }
    }
    public com.acmerobotics.roadrunner.Action autoClawStateOpen() {
        return new doAutoClawStateOpen();
    }
    public class doAutoClawStateClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (specimenClawDone) {
                specimenClawDone = false;
                specimenClawTime.reset();
                clawStateClose();
                return true;
            }
            if (specimenClawTime.milliseconds() < SPECIMENCLAW_OPEN_TIME) {
                return true;
            }
            specimenClawDone = true;
            return false;
        }
    }

    public com.acmerobotics.roadrunner.Action autoClawStateClose() {
        return new doAutoClawStateClose();
    }
    */
    public void goToNextSpecimenState(){
        telemetry.addData("Status", "Shoulder Moved");
        switch(shoulderPosition){
            case ENTER:
                scoreRequested=true;
                latchTime.reset();
                shoulderPosition = ShoulderState.SCORE;
                rightShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //rightShoulder.setTargetPosition(R_SHOULDER_SCORE_POS);
                //rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setPower(LATCH_SPEED);
                //leftShoulder.setPosition(L_SHOULDER_ENTER_POS);
                //rightShoulder.setPosition(R_SHOULDER_COLLECT_POS);
                break;
            case STOW:
            case COLLECT:
                scoreRequested = false;
                shoulderPosition = ShoulderState.ENTER;
                rightShoulder.setTargetPosition(R_SHOULDER_ENTER_POS);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setPower(MOVE_SPEED);
                //leftShoulder.setPosition(L_SHOULDER_COLLECT_POS);
                //rightShoulder.setPosition(R_SHOULDER_ENTER_POS);
                break;
            case SCORE:
                scoreRequested = false;
                shoulderPosition = ShoulderState.COLLECT;
                rightShoulder.setTargetPosition(R_SHOULDER_COLLECT_POS);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightShoulder.setPower(MOVE_SPEED);
                break;
            default:
                telemetry.addData("Shoulder", "went wrong");
        }
    }

    public void goToCollectArmState(){
        scoreRequested = false;
        shoulderPosition = ShoulderState.COLLECT;
        rightShoulder.setTargetPosition(R_SHOULDER_COLLECT_POS);
        rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightShoulder.setPower(MOVE_SPEED);
    }

    public ShoulderState getShoulderState(){
        return shoulderPosition;
    }
    public void resetSpecimenEncoders(){
        rightShoulder.setPower(0.0);
        rightShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void resetSpecimenWithoutLimits() {
        rightShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShoulder.setPower(R_SHOULDER_RESET_POWER);
    }
    public void stowSpecimenArm(){
        shoulderPosition = ShoulderState.STOW;
        rightShoulder.setTargetPosition(R_SHOULDER_STOW_POS);
        rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightShoulder.setPower(MOVE_SPEED);
        //rightShoulder.setPosition(R_SHOULDER_STOW_POS);
    }

    public void disableSpecimenArm(){
        shoulderPosition = ShoulderState.STOW;
        rightShoulder.setPower(0);
    }
    /*
    public com.acmerobotics.roadrunner.Action nextSpecimenStateAction() {
        return new SpecimenArmNextState();
    }

    public class SpecimenArmNextState implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (specimenArmDone) {
                specimenArmDone = false;
                specimenArmTime.reset();
                goToNextSpecimenState();
                return true;
            }
            if (specimenArmTime.milliseconds() < SPECIMENARM_MOVE_TIME) {
                return true;
            }
            specimenArmDone = true;
            return false;
        }
    }*/
}

