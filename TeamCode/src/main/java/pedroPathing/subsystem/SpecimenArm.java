package pedroPathing.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
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
    private ServoImplEx claw = null;
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
    public static int R_SHOULDER_COLLECT_POS = 480; //470; //485; //500;//495;//490;//500; //440; //125;
    public static int R_SHOULDER_ENTER_POS = 130; //120;//113;//125; //490;
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
    public static double SPECIMENCLAW_OPEN_TIME = 200.0; //1000.0;
    public static double SPECIMENARM_SCORE_TIME = 150.0;
    public static double SPECIMENARM_COLLECT_THRESHOLD = 10.0;
    public static double SCORE_LATCH_TIME = 1.0;
    public static double LATCH_SPEED = 0.8;
    public static double MOVE_SPEED = 0.5;
    public static double COLLECT_SPEED =0.0;
    public static double R_SHOULDER_RESET_POWER = -0.2;



    public SpecimenArm(HardwareMap hm, Telemetry tele, boolean resetMotors) {
        hardwareMap = hm;
        telemetry = tele;
        telemetry.addData("Specimen Arm Status", "initializing");


        leftShoulder = hardwareMap.get(Servo.class, "left_shoulder"); // Intake: 11 even
        claw = hardwareMap.get(ServoImplEx.class, "claw");
        rightShoulder = hardwareMap.get(DcMotor.class, "right_shoulder"); // Ready: 26.75
        leftShoulderEx = hardwareMap.get(ServoImplEx.class, "left_shoulder");
        if (resetMotors){
            rightShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        rightShoulder.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(){
        if(scoreRequested && latchTime.milliseconds()> SPECIMENARM_SCORE_TIME){
            scoreRequested = false;
            //rightShoulder.setTargetPosition(R_SHOULDER_SCORE_POS);
            //rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightShoulder.setPower(0);
        }
        if (shoulderPosition != ShoulderState.ENTER && rightShoulder.getCurrentPosition() +SPECIMENARM_COLLECT_THRESHOLD > R_SHOULDER_COLLECT_POS){
            rightShoulder.setPower(0);
            shoulderPosition = ShoulderState.COLLECT;
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
    public void makeLimp(){
        rightShoulder.setPower(0);
        claw.setPwmDisable();
    }

    public class doAutoClawStateOpen extends CommandBase {
        boolean isDone = false;

        public doAutoClawStateOpen() {
        }
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            if (specimenClawDone) {
                specimenClawDone = false;
                specimenClawTime.reset();
                clawStateOpen();
                isDone = false;
                return;
            }
            if (specimenClawTime.milliseconds() < SPECIMENCLAW_OPEN_TIME) {
                isDone = false;
                return;
            }
            specimenClawDone = true;
            isDone = true;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }
    public class doAutoClawStateClose extends CommandBase {
        boolean isDone = false;

        public doAutoClawStateClose() {
        }
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            if (specimenClawDone) {
                specimenClawDone = false;
                specimenClawTime.reset();
                clawStateClose();
                isDone = false;
                return;
            }
            if (specimenClawTime.milliseconds() < SPECIMENCLAW_OPEN_TIME) {
                isDone = false;
                return;
            }
            specimenClawDone = true;
            isDone = true;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    /*public void shoulderPowerPosition(){
        if (rightShoulder.getCurrentPosition() == R_SHOULDER_COLLECT_POS) {
            rightShoulder.setPower(0);
        }else if (rightShoulder.getCurrentPosition() == R_SHOULDER_ENTER_POS){
            rightShoulder.setPower(LATCH_SPEED);
        }else{
            rightShoulder.setPower(MOVE_SPEED);
        }
    }*/

    public void goToNextSpecimenState(){
        telemetry.addData("Status", "Shoulder Moved");
        switch(shoulderPosition){
            case ENTER:
                rightShoulder.setPower(LATCH_SPEED);
                scoreRequested=true;
                latchTime.reset();
                shoulderPosition = ShoulderState.SCORE;
                rightShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //rightShoulder.setTargetPosition(R_SHOULDER_SCORE_POS);
                //rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //leftShoulder.setPosition(L_SHOULDER_ENTER_POS);
                //rightShoulder.setPosition(R_SHOULDER_COLLECT_POS);
                break;
            case STOW:
            case COLLECT:
                //if (rightShoulder.getCurrentPosition() < R_SHOULDER_COLLECT_POS){
                    rightShoulder.setPower(MOVE_SPEED);
                    scoreRequested = false;
                    shoulderPosition = ShoulderState.ENTER;
                    rightShoulder.setTargetPosition(R_SHOULDER_ENTER_POS);
                    rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //}else{
                    //rightShoulder.setPower(0);
                //}
                //leftShoulder.setPosition(L_SHOULDER_COLLECT_POS);
                //rightShoulder.setPosition(R_SHOULDER_ENTER_POS);
                break;
            case SCORE:
                rightShoulder.setPower(MOVE_SPEED);
                scoreRequested = false;
                shoulderPosition = ShoulderState.COLLECT;
                rightShoulder.setTargetPosition(R_SHOULDER_COLLECT_POS);
                rightShoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public class SpecimenArmNextState extends CommandBase {
        boolean isDone = false;

        public SpecimenArmNextState() {
        }

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            if (specimenArmDone) {
                specimenArmDone = false;
                specimenArmTime.reset();
                goToNextSpecimenState();
                isDone = false;
                return;
            }
            if (specimenArmTime.milliseconds() < SPECIMENARM_SCORE_TIME) {
                isDone = false;
            }
            specimenArmDone = true;
            if (rightShoulder.getTargetPosition() == R_SHOULDER_COLLECT_POS) {
                rightShoulder.setPower(0);
            }
            isDone = true;
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }
}

