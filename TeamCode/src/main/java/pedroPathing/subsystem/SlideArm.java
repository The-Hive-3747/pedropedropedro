package pedroPathing.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Extending the arm, active intake, pivoting the arm, rotating the wrist. This is the arm that
 * slides out the wrist AKA the active intake and can score into the basket, after everything it
 * can slide back in again.
 */
@Config
public class SlideArm {
    private DcMotor pivotMotor = null;
    private DcMotor leftSlideMotor = null;
    private DcMotor rightSlideMotor = null;
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;
    private ElapsedTime intakeTime = new ElapsedTime();
    private ElapsedTime wristTime = new ElapsedTime();
    private static ElapsedTime autoIntakeTime = new ElapsedTime();
    private static ElapsedTime sleepTime = new ElapsedTime();
    private boolean autoIntakeStarted = false;
    private boolean sleepStarted = false;
    private boolean pivotUpRequested = false;
    private boolean pivotDownRequested = false;
    public boolean hangRequested = false;
    private boolean doRelease = false;
    private String servoStatus = "";
    public enum PivotState {UP, DOWN}
    public enum WristState {READY, GATHER, SCORE}
    private PivotState pivotPosition = PivotState.DOWN;
    private WristState wristPosition = WristState.READY;
    //public static int PIVOT_ENTER_POSITION = 400; //200;
    public static int PIVOT_UP_POSITION = 1400; //1500; //1350; //1245; measured on old motor//1031 actual;//335
    public static int PIVOT_DOWN_POSITION = 2;
    public static int PIVOT_HANG_POSITION = 1145; //1140; //1284;
    public static double PIVOT_POWER_UP = 0.7;
    public static double PIVOT_POWER_ENTER_UP = 0.6;
    public static double PIVOT_POWER_DOWN = 0.7; //0.05;
    public static double PIVOT_HANG_POWER = 0.6;
    public static int PIVOT_UP_THRESHOLD = 900;
    public static int PIVOT_TOLERANCE = 10;
    public static double PIVOT_POWER_HOLD = 0.3; //0.2;
    public static double SLIDE_POWER_TEST = 0.1;
    public static double SLIDE_POWER = 0.9;//0.2;nex
    public static double SLIDE_POWER_RELEASE = 0.1;
    public static double SLIDE_POWER_HANG_HOLD = 0.6;
    public static int SLIDE_SLOW_STEP = 20;
    public static int SLIDE_STEP = 240;//60;
    public static int SLIDE_LEFT_HORIZ_LEGAL_LIMIT_TICKS = 1472;//2027;
    public static int SLIDE_RIGHT_HORIZ_LEGAL_LIMIT_TICKS = 1472;//2027;
    public static int SLIDE_LEFT_HEIGHT_SCORE_TICKS = 1936; //3568;
    public static int SLIDE_RIGHT_HEIGHT_SCORE_TICKS = 1936; //3569;
    public static int SLIDE_LEFT_HANG_LIMIT = 2319;//3962;
    public static int SLIDE_RIGHT_HANG_LIMIT = 2319;//3967;
    public static int SLIDE_LEFT_HANG_HEIGHT = 984;//1450;old hang for 312 motor
    public static int SLIDE_RIGHT_HANG_HEIGHT = 984;//1450;old hang for 312 motor
    public static int SLIDE_LEFT_MIN_LIMIT_BUFFER =10;
    public static int SLIDE_BACK_MIN_LIMIT_BUFFER=10;
    public static int SLIDE_FRONT_STOP_TICKS = SLIDE_LEFT_HEIGHT_SCORE_TICKS;
    public static int SLIDE_BACK_STOP_TICKS = SLIDE_RIGHT_HEIGHT_SCORE_TICKS; //3967;
    public static int SLIDE_FRONT_NO_DOWN_TICKS = 1000;
    public static int SLIDE_TOLERANCE = 50;
    public static double INTAKE_DISTANCE_STOP_CM = 7.35;
    public static double WRIST_STOW_POS = 0.89; //0.0; old servo
    public static double WRIST_READY_POS = 0.4; //0.33; //0.12; old servo
    public static double WRIST_GATHER_POS = 0.12; // 0.15;
    public static double WRIST_SCORE_POS = 0.5; //1; // 0.7 0.5
    public static double AUTO_INTAKE_THRESHOLD = 1500.0;
    public static double AUTO_INTAKE_PICKUP_THRESHOLD = 500.0;
    public static double INTAKE_LEFT_POWER = -0.8;
    public static double INTAKE_RIGHT_POWER = -0.8;
    public static double INTAKE_LEFT_POWER_SCORE = 0.8;
    public static double INTAKE_RIGHT_POWER_SCORE = 0.8;
    private static double SLIDE_POWER_SCORE_HOLD = 0.3;
    private static double LUMINANCE_RED = 0.299;
    private static double LUMINANCE_GREEN = 0.587;
    private static double LUMINANCE_BLUE = 0.114;
    private static double LUMINANCE_PB = 0.564;
    public static double LUMINANCE_PR = 0.713;
    public static double LUMINANCE_RED_CR_MIN = -34.0;
    public static double LUMINANCE_YELLOW_CR_MAX = -34.0;
    public static double LUMINANCE_BLUE_CB_MIN = -16.0;
    public static double SLEEP_TIME = 5000.0;
    public static double PIVOT_RESET_POWER = -0.1;
    public static double SLIDE_RESET_POWER = -0.2;

    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    //private RevColorSensorV3 intakeColor = null;
    private Servo wrist = null;
    private double afterDetectTimeMS = 180.0;



    public SlideArm(HardwareMap hm, Telemetry tele) {
        hardwareMap = hm;
        telemetry = tele;
        telemetry.addData("Slide arm status", "initializing");

        pivotMotor = hardwareMap.get(DcMotor.class,"pivot_motor");
        leftSlideMotor = hardwareMap.get(DcMotor.class,"left_slide_motor" );
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_slide_motor");
        leftIntake = hardwareMap.get(CRServo.class, "intake_left");
        rightIntake = hardwareMap.get(CRServo.class, "intake_right");
        //intakeColor = hardwareMap.get(RevColorSensorV3.class, "intake_color");
        wrist = hardwareMap.get(Servo.class, "wrist");

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //pivotMotor.setTargetPosition(0);
        //leftSlideMotor.setTargetPosition(0);
        //rightSlideMotor.setTargetPosition(0);

        //pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //TODO: set polarity of motors
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Slide arm status", "init complete");
    }

    public void update(){
        if (pivotUpRequested && pivotMotor.getCurrentPosition() +PIVOT_TOLERANCE > PIVOT_UP_POSITION ) {
            telemetry.addData("PivotUp", "Completed");
            pivotUpRequested = false;
            pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pivotMotor.setPower(PIVOT_POWER_HOLD);
        }
        if (pivotDownRequested && pivotMotor.getCurrentPosition() -PIVOT_TOLERANCE < PIVOT_DOWN_POSITION ) {
            telemetry.addData("PivotDown", "Completed");
            pivotDownRequested = false;
            pivotMotor.setPower(0);
        }
        if (doRelease){
            int LIMIT = SLIDE_LEFT_HANG_LIMIT;
            boolean isLeftDone = false;
            boolean isRightDone = false;
            if(leftSlideMotor.getCurrentPosition() <= LIMIT - SLIDE_TOLERANCE){
                leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftSlideMotor.setPower(SLIDE_POWER_RELEASE);
            }else{
                leftSlideMotor.setPower(0);
                isLeftDone = true;
            }
            if(rightSlideMotor.getCurrentPosition() <= LIMIT - SLIDE_TOLERANCE) {
                rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightSlideMotor.setPower(SLIDE_POWER_RELEASE);
            }else{
                rightSlideMotor.setPower(0);
                isRightDone = true;
            }
        }

    }

    public void pivotHang(){
        hangRequested = true;
        pivotMotor.setTargetPosition(PIVOT_HANG_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(PIVOT_HANG_POWER);
    }

    public void hangHold(){
        if (!hangRequested){
            return;
        }
        leftSlideMotor.setTargetPosition(SLIDE_LEFT_HANG_HEIGHT);
        rightSlideMotor.setTargetPosition(SLIDE_RIGHT_HANG_HEIGHT);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(SLIDE_POWER_HANG_HOLD);
        rightSlideMotor.setPower(SLIDE_POWER_HANG_HOLD);
    }
    public void hangRelease(){
        if (!hangRequested){
            return;
        }
        doRelease = true;
    }

    /**
     *
     */
    public void pivotUp(){
//        switch(pivotPosition) {
            /*case GATHER:
                hangRequested = false;
                pivotPosition = PivotState.UP;
                pivotMotor.setTargetPosition(PIVOT_UP_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_ENTER_UP);
                pivotUpRequested = true;
                break;
            case ENTER:
                hangRequested = false;
                pivotPosition = PivotState.UP;
                pivotMotor.setTargetPosition(PIVOT_UP_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_UP);
                pivotUpRequested = true;
                break;
                */
//
//            case GATHER:
//            case UP:
        hangRequested = false;
        pivotMotor.setTargetPosition(PIVOT_UP_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(PIVOT_POWER_UP);
        pivotUpRequested = true;
//                break;
//            default:
//                telemetry.addData("Problem", "Pivot Up Invalid Position");
//        }
    }
    /**
     *
     */
    public void pivotDown(){
        /*switch(pivotPosition) {
            case UP:
                hangRequested = false;
                if(getFrontSlideTicks()>SLIDE_FRONT_NO_DOWN_TICKS){
                    break;
                }
                pivotPosition = PivotState.GATHER;
                pivotMotor.setTargetPosition(PIVOT_GATHER_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_DOWN);
                pivotDownRequested = true;
                break;
            /*case ENTER:
                hangRequested = false;
                pivotPosition = PivotState.GATHER;
                pivotMotor.setTargetPosition(PIVOT_GATHER_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_DOWN);
                pivotDownRequested = true;
                break;
            case GATHER:*/
        if(getFrontSlideTicks()>SLIDE_FRONT_NO_DOWN_TICKS){
            return;
        }
        hangRequested = false;
        pivotMotor.setTargetPosition(PIVOT_DOWN_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(PIVOT_POWER_DOWN);
        pivotDownRequested = true;
//                break;
//            default:
//                telemetry.addData("Problem", "Pivot Down Invalid Position");
//        }
    }

    public void makeLimp() {
        pivotMotor.setPower(0);
        rightSlideMotor.setPower(0);
        leftSlideMotor.setPower(0);
    }
    public int getPivotTicks(){
        return pivotMotor.getCurrentPosition();
    }
    public PivotState getPivotState() {
        return pivotPosition;
    }
    public void slowMoveMotors(boolean isRetracting){
        if (isRetracting){
            leftSlideMotor.setTargetPosition(leftSlideMotor.getCurrentPosition()-SLIDE_SLOW_STEP);
            rightSlideMotor.setTargetPosition(leftSlideMotor.getCurrentPosition()-SLIDE_SLOW_STEP);
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlideMotor.setPower(SLIDE_POWER_TEST);
            rightSlideMotor.setPower(SLIDE_POWER_TEST);
        } else{
            rightSlideMotor.setTargetPosition(rightSlideMotor.getCurrentPosition()+SLIDE_SLOW_STEP);
            leftSlideMotor.setTargetPosition(rightSlideMotor.getCurrentPosition()+SLIDE_SLOW_STEP);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setPower(SLIDE_POWER_TEST);
            leftSlideMotor.setPower(SLIDE_POWER_TEST);
        }

    }
    public void resetSlideEncoders(){
        leftSlideMotor.setPower(0.0);
        rightSlideMotor.setPower(0.0);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void retractWithoutLimits(){
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setPower(SLIDE_RESET_POWER);
        rightSlideMotor.setPower(SLIDE_RESET_POWER);
    }
    public void resetPivotEncoders(){
        pivotMotor.setPower(0.0);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void pivotDownWithoutLimits(){
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setPower(PIVOT_RESET_POWER);
    }
    public void stopSliding(){
        //leftSlideMotor.setPower(0);
        //rightSlideMotor.setPower(0);
        leftSlideMotor.setTargetPosition(leftSlideMotor.getCurrentPosition());
        rightSlideMotor.setTargetPosition(rightSlideMotor.getCurrentPosition());
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
        rightSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);

    }
    public boolean slideUpOneStep(){

        int LIMIT = SLIDE_LEFT_HORIZ_LEGAL_LIMIT_TICKS;
        boolean isLeftDone = false;
        boolean isRightDone = false;
        if(pivotMotor.getCurrentPosition() > PIVOT_UP_THRESHOLD) {
            LIMIT = SLIDE_LEFT_HEIGHT_SCORE_TICKS;
        }
        if (hangRequested){
            LIMIT = SLIDE_LEFT_HANG_LIMIT;
        }

        if(leftSlideMotor.getCurrentPosition() <= LIMIT - SLIDE_TOLERANCE){
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlideMotor.setPower(SLIDE_POWER);
        }else{
            leftSlideMotor.setPower(0);
            isLeftDone = true;
        }
        if(rightSlideMotor.getCurrentPosition() <= LIMIT - SLIDE_TOLERANCE) {
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setPower(SLIDE_POWER);
        }else{
            rightSlideMotor.setPower(0);
            isRightDone = true;
        }
        return !isLeftDone || !isRightDone;


        /*
        int targetPos = frontSlideMotor.getCurrentPosition()+SLIDE_STEP;
        if (targetPos>SLIDE_FRONT_STOP_TICKS){
            frontSlideMotor.setTargetPosition(SLIDE_FRONT_STOP_TICKS);
            backSlideMotor.setTargetPosition(SLIDE_BACK_STOP_TICKS);
            frontSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontSlideMotor.setPower(SLIDE_POWER);
            backSlideMotor.setPower(SLIDE_POWER);
        } else{
            frontSlideMotor.setTargetPosition(frontSlideMotor.getCurrentPosition()+SLIDE_STEP);
            backSlideMotor.setTargetPosition(backSlideMotor.getCurrentPosition()+SLIDE_STEP);
            frontSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontSlideMotor.setPower(SLIDE_POWER);
            backSlideMotor.setPower(SLIDE_POWER);
        }
        */
    }


    public boolean slideDownOneStep(){
        doRelease = false;
        int LIMIT = SLIDE_LEFT_MIN_LIMIT_BUFFER;
        boolean isLeftDone = false;
        boolean isRightDone = false;
        if(leftSlideMotor.getCurrentPosition() >= LIMIT + SLIDE_TOLERANCE){
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlideMotor.setPower(-SLIDE_POWER);
        }else{
            leftSlideMotor.setPower(0);
            isLeftDone = true;
        }
        if(rightSlideMotor.getCurrentPosition() >= LIMIT + SLIDE_TOLERANCE) {
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setPower(-SLIDE_POWER);
        }else{
            rightSlideMotor.setPower(0);
            isRightDone = true;
        }
        return !isLeftDone || !isRightDone;
        /*int targetPos = leftSlideMotor.getCurrentPosition()-SLIDE_STEP;
        if (targetPos<=0){
            leftSlideMotor.setTargetPosition(SLIDE_LEFT_MIN_LIMIT_BUFFER);
            rightSlideMotor.setTargetPosition(SLIDE_BACK_MIN_LIMIT_BUFFER);
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlideMotor.setPower(SLIDE_POWER);
            rightSlideMotor.setPower(SLIDE_POWER);
        } else{
            leftSlideMotor.setTargetPosition(leftSlideMotor.getCurrentPosition()-SLIDE_STEP);
            rightSlideMotor.setTargetPosition(rightSlideMotor.getCurrentPosition()-SLIDE_STEP);
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlideMotor.setPower(SLIDE_POWER);
            rightSlideMotor.setPower(SLIDE_POWER);
        } */
    }
    public int getFrontSlideTicks() {
        return leftSlideMotor.getCurrentPosition();
    }
    public int getBackSlideTicks() {
        return rightSlideMotor.getCurrentPosition();
    }
    /*public void activateIntakeWithSensor(){
        if (intakeColor.getDistance(DistanceUnit.CM) > INTAKE_DISTANCE_STOP_CM){
            leftIntake.setPower(INTAKE_LEFT_POWER);
            rightIntake.setPower(INTAKE_RIGHT_POWER);
            servoStatus = "Intake with Sensor Active";
            intakeTime.reset();
        } else {
            if (intakeTime.milliseconds() > afterDetectTimeMS && isTeamColorBlock()) {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                servoStatus = "Inactive";
            } else {
                servoStatus = "Intake sensed object and waiting";
            }
        }
    }*/
    /*public SampleColor detectColor(){
        if (LUMINANCE_BLUE_CB_MIN < getLuminancePb()){
            return SampleColor.SAMPLE_BLUE;
        }
        else if (LUMINANCE_RED_CR_MIN < getLuminancePr()) {
            return SampleColor.SAMPLE_RED;
        }
        return SampleColor.SAMPLE_YELLOW;
    }
    public boolean isKeeperBlock(){
        if (TeleOpComp.TeamColor.TEAM_BLUE==TeleOpComp.HIVE_COLOR
        && SampleColor.SAMPLE_BLUE== detectColor()){
            return true;
        }
        else if (TeleOpComp.TeamColor.TEAM_RED==TeleOpComp.HIVE_COLOR
        && SampleColor.SAMPLE_RED== detectColor()){
            return true;
        }
        else if (SampleColor.SAMPLE_YELLOW== detectColor()){
            return true;
        }
        return false;
    }
    public boolean isTeamColorBlock(){
        if (TeleOpComp.TeamColor.TEAM_BLUE==TeleOpComp.HIVE_COLOR
                && SampleColor.SAMPLE_BLUE== detectColor()){
            return true;
        }
        else if (TeleOpComp.TeamColor.TEAM_RED==TeleOpComp.HIVE_COLOR
                && SampleColor.SAMPLE_RED== detectColor()){
            return true;
        }
        return false;
    }*/
    public void activateIntakeWithoutSensor(){
        leftIntake.setPower(INTAKE_LEFT_POWER);
        rightIntake.setPower(INTAKE_RIGHT_POWER);
        servoStatus = "Intake Active";
    }
    public void stopIntake(){
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        servoStatus = "Inactive";
    }
    public void reverseIntakeWithoutSensor(){
        leftIntake.setPower(INTAKE_LEFT_POWER_SCORE);
        rightIntake.setPower(INTAKE_RIGHT_POWER_SCORE);
        servoStatus = "Intake Reversed";

    }
    public String getIntakeStatus(){
        return servoStatus;
    }
    public void setWristToStow(){
        wrist.setPosition(WRIST_STOW_POS);
    }
    public void setWristToReady(){
        wrist.setPosition(WRIST_READY_POS);
        wristPosition = WristState.READY;
    }

    public void nextWristPosition(){
        switch(wristPosition) {
            case GATHER:
                wrist.setPosition(WRIST_GATHER_POS);
                wristPosition = WristState.SCORE;
                break;
            case SCORE:
                wrist.setPosition(WRIST_SCORE_POS);
                wristPosition = WristState.READY;
                break;
            case READY:
            default:
                wrist.setPosition(WRIST_READY_POS);
                wristPosition = WristState.GATHER;
        }
    }
    /*public void moveWristLeft(){
        double pos = wrist.getPosition();
        if (pos > WRIST_GATHER_POS && ((int)(wristTime.milliseconds()) / 100) % 5 == 0) {
            pos -= 0.5; // 0.1 0.01
            wrist.setPosition(pos);
        }
    }
    public void moveWristRight(){
        double pos = wrist.getPosition();
        if (pos < WRIST_SCORE_POS && ((int)(wristTime.milliseconds()) / 100) % 5 == 0) {
            pos += 0.05; // 0.1 0.01
            wrist.setPosition(pos);
        }
    }*/

    /*public double getIntakeDistaceCM(){
        return intakeColor.getDistance(DistanceUnit.CM);
    }
    public double getLuminancePb() {
        return LUMINANCE_PB*(intakeColor.blue())-(intakeColor.red()*LUMINANCE_RED+
                LUMINANCE_GREEN*intakeColor.green()+LUMINANCE_BLUE*intakeColor.blue());
    }
    public double getLuminancePr() {
        return LUMINANCE_PR*(intakeColor.red())-(intakeColor.red()*LUMINANCE_RED+
                LUMINANCE_GREEN*intakeColor.green()+LUMINANCE_BLUE*intakeColor.blue());
    }*/
    public void addSlideTelemetry(){
        telemetry.addData("Left Slide Ticks", leftSlideMotor.getCurrentPosition());
        telemetry.addData("Left Slide Ticks", rightSlideMotor.getCurrentPosition());
        telemetry.addData("Pivot Ticks", pivotMotor.getCurrentPosition());
        telemetry.addData("PivotUpRequested", pivotUpRequested);
        telemetry.addData("PivotDownRequested", pivotDownRequested);
        telemetry.addData("PivotState", pivotPosition);
    }
    /*public void retractPivotUp() {
        slideDownOneStep();
        pivotPosition = PivotState.UP;
        pivotUp();
        slideUpOneStep();
    }
    public void retractPivotDown() {
        slideDownOneStep();
        pivotPosition = PivotState.ENTER;
        pivotDown();
        slideUpOneStep();

    }*/
    /*
    public class RetractSlideArm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //retractSlideArm();
            telemetry.addData("Slide", "Retracting");
            telemetry.update();
            return slideDownOneStep();
        }
    }*/
    /*public Action retractSlideArmAction(){
        return new RetractSlideArm();
    }
    public class ExtendSlideArm implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //extendSlideArm();
            telemetry.addData("Slide", "Extending");
            telemetry.update();
            return slideUpOneStep();
        }
    }
    /*public Action extendSlideArmAction() {
        return new ExtendSlideArm();
    }*/
    /*
    public class PivotSlideArmUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //pivotSlideArmUp();
            if (pivotMotor.getCurrentPosition() +PIVOT_TOLERANCE > PIVOT_UP_POSITION) {
                telemetry.addData("PivotUp", "Completed");
                telemetry.update();
                pivotUpRequested = false;
                pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pivotMotor.setPower(PIVOT_POWER_HOLD);
                return false;
            }
            else if (!pivotUpRequested){
                pivotUpRequested = true;
                pivotPosition = PivotState.UP;
                telemetry.addData("PivotUp", "Starting");
                telemetry.update();
                pivotUp();
            }
            telemetry.addData("PivotUp", "Busy");
            telemetry.update();
            return true;
        }
    }*/
    /*public Action pivotSlideArmUpAction() {
        return new PivotSlideArmUp();
    }*/
    /*public class PivotSlideArmUpToEnter implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //pivotSlideArmUp();
            if (pivotMotor.getCurrentPosition() + PIVOT_TOLERANCE > PIVOT_ENTER_POSITION) {
                telemetry.addData("PivotEnter", "Completed");
                telemetry.update();
                pivotUpRequested = false;
                pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pivotMotor.setPower(PIVOT_POWER_HOLD);
                return false;
            }
            else if (!pivotUpRequested){
                pivotUpRequested = true;
                pivotPosition = PivotState.DOWN;
                telemetry.addData("PivotEnter", "Starting");
                telemetry.update();
                pivotUp();
            }
            telemetry.addData("PivotEnter", "Busy");
            telemetry.update();
            return true;
        }
    }
    public Action pivotSlideArmUpToEnterAction() {
        return new PivotSlideArmUpToEnter();
    }
    public class PivotSlideArmDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //pivotSlideArmDown();
            if (pivotMotor.getCurrentPosition() - PIVOT_TOLERANCE< PIVOT_DOWN_POSITION) {
                telemetry.addData("PivotDown", "Completed");
                telemetry.update();
                pivotDownRequested = false;
                return false;
            }
            else if (!pivotDownRequested){
                pivotDownRequested = true;
                pivotPosition = PivotState.DOWN;
                telemetry.addData("PivotDown", "Starting");
                telemetry.update();
                pivotDown();
            }
            telemetry.addData("PivotDown", "Busy");
            telemetry.update();
            return true;
        }
    }*/
    /*public Action pivotSlideArmDownAction() {
        return new PivotSlideArmDown();

    }*/
    /*public class PivotSlideArmDownToEnter implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //pivotSlideArmDown();
            if (pivotMotor.getCurrentPosition() - PIVOT_TOLERANCE<PIVOT_ENTER_POSITION) {
                telemetry.addData("PivotDown", "Completed");
                telemetry.update();
                pivotDownRequested = false;
                return false;
            }
            else if (!pivotDownRequested){
                pivotDownRequested = true;
                pivotPosition = PivotState.UP;
                telemetry.addData("PivotDown", "Starting");
                telemetry.update();
                pivotDown();
            }
            telemetry.addData("PivotDown", "Busy");
            telemetry.update();
            return true;
        }
    }
    public Action pivotSlideArmDownToEnterAction() {
        return new PivotSlideArmDownToEnter();
    }*/
    /*public class IntakeScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //reverseIntakeScore();
            if (!autoIntakeStarted) {
                autoIntakeStarted = true;
                autoIntakeTime.reset();
                activateIntakeWithoutSensor();
                return true;
            }
            else if (autoIntakeTime.milliseconds() < AUTO_INTAKE_THRESHOLD) {
                activateIntakeWithoutSensor();
                return true;
            }
            autoIntakeStarted = false;
            stopIntake();
            return false;
        }
    }
    public Action intakeScoreAction() {
        return new IntakeScore();
    }
    public class IntakeSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //reverseIntakeScore();
            if (!autoIntakeStarted) {
                autoIntakeStarted = true;
                autoIntakeTime.reset();
                activateIntakeWithoutSensor();
                return true;
            }
            else if (autoIntakeTime.milliseconds() < AUTO_INTAKE_PICKUP_THRESHOLD) {
                activateIntakeWithoutSensor();
                return true;
            }
            autoIntakeStarted = false;
            stopIntake();
            return false;
        }
    }
    public Action intakeSampleAction() {
        return new IntakeSample();
    }*/
    /*public class Gather implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //gather();
            return false;
        }
    }
    public Action gatherAction() {
        return new Gather();
    }*/
    /*
    public class wristReady implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_READY_POS);
            return false;
        }
    }
    public Action wristReadyAction() {
        return new wristReady();
    }
    public class wristGather implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_GATHER_POS);
            return false;
        }
    }
    public Action wristGatherAction() {
        return new wristGather();
    }
    public class wristScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_SCORE_POS);
            return false;
        }
    }
    public Action wristScoreAction() {
        return new wristScore();
    }
    public class wristStow implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_STOW_POS);
            return false;
        }
    }
    public Action wristStowAction() {
        return new wristStow();
    }
    public class Sleep250Millisec implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //reverseIntakeScore();
            if (!sleepStarted) {
                sleepStarted = true;
                sleepTime.reset();
                return true;
            }
            else if (sleepTime.milliseconds() < 250) {
                return true;
            }
            sleepStarted = false;
            return false;
        }
    }
    public Action sleep250MillisAction() {
        return new Sleep250Millisec();
    }
    public class Sleep750Millisec implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //reverseIntakeScore();
            if (!sleepStarted) {
                sleepStarted = true;
                sleepTime.reset();
                return true;
            }
            else if (sleepTime.milliseconds() < 750) {
                return true;
            }
            sleepStarted = false;
            return false;
        }
    }
    public Action sleep750MillisAction() {
        return new Sleep750Millisec();
    }
    */

}
