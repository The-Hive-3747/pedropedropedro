package pedroPathing.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.TeleOpComp;
import pedroPathing.TeleOpComp.SampleColor;

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
    private DcMotor pivotEncoder = null;
    private HardwareMap hardwareMap = null;
    private Telemetry telemetry = null;
    private ServoImplEx rightDerailer = null;
    private ServoImplEx leftDerailer = null;
    private CRServo rightTensioner = null;
    private CRServo leftTensioner = null;
    private ElapsedTime intakeTime = new ElapsedTime();
    private ElapsedTime wristTime = new ElapsedTime();
    private ElapsedTime autoPivotTime = new ElapsedTime();
    private static ElapsedTime autoIntakeTime = new ElapsedTime();
    private static ElapsedTime sleepTime = new ElapsedTime();
    private static ElapsedTime tensionTime = new ElapsedTime();
    private boolean autoIntakeStarted = false;
    private boolean sleepStarted = false;
    private boolean pivotUpRequested = false;
    private boolean pivotDownRequested = false;
    private boolean pivotIsResting = true;
    public boolean hangRequested = false;
    public boolean unzeroed = false;
    private boolean hasDerailed = false;
    public boolean firstHangRequested = false;
    private boolean doRelease = false;
    private boolean isHolding = false;
    private boolean tensionDone = false;
    private boolean tensionStarted = false;
    private boolean tensionDownStarted = false;
    private boolean isPivotingDownAndRetracting = false;
    public boolean tensionRightSlide = true;
    public boolean loosenRightSlide = true;
    private String servoStatus = "";
    public enum PivotState {UP, DOWN, MANUAL}
    public enum PivotHangState {UP, READY, BAR, DOWN, HANG}
    public enum WristState {READY, GATHER, SCORE, STOW}
    private PivotState pivotPosition = PivotState.DOWN;
    private PivotHangState pivotHangPosition = PivotHangState.UP;
    private WristState wristPosition = WristState.READY;
    //public static int PIVOT_ENTER_POSITION = 400; //200;
    private boolean tensioningDone = false;
    public static int PIVOT_UP_POSITION = 915;//924;//954;//930;//740;//776;//1400; //1500; //1350; //1245; measured on old motor//1031 actual;//335
    public static int PIVOT_DOWN_POSITION = 2;
    public static int PIVOT_HANG_BAR_POSITION = 400;
    public static int PIVOT_THROUGHBORE_DOWN_POSITION = 6;
    public static int PIVOT_THROUGHBORE_THRESHOLD = 10;
    public static int PIVOT_THROUGHBORE_UP_MAX = 2220;
    public static int PIVOT_DOWN_FIRST_POSITION = 640;
    public static int PIVOT_HANG_PULL_UP_POSITION = 100;
    public static int PIVOT_HANG_POSITION = 850;//594; //514; //491; //590; //710; //680; //590; //610;//490;//1145; //1140; //1284;
    public static int PIVOT_TALL_HANG_POSITION = 750;//594; //514; //491; //590; //710; //680; //590; //610;//490;//1145; //1140; //1284;
    public static int PIVOT_HANG_LEVEL_TWO_POSITION = 200; //100; //0;
    public static int PIVOT_HANG_LEVEL_TWO_TIP = 580;//330;
    public static int PIVOT_HANG_LEVEL_TWO_HOOK = 550;
    public static int PIVOT_HANG_LEVEL_THREE_HOLD_MOVE = -10;
    public static int PIVOT_DOWN_WHILE_HANG = 466;
    public static int PIVOT_DOWN_FOR_HANG = 390;
    public boolean backupUsed = false;
    public static double PIVOT_POWER_UP = 0.95;//0.85; //0.6; //0.7;
    public static double PIVOT_POWER_ENTER_UP = 0.6;
    public static double PIVOT_POWER_DOWN = 0.5;//0.6;//0.7;//0.5;//0.3; //0.7; //0.05;
    public static double PIVOT_POWER_LIFT_BODY = 0.8;
    public static double PIVOT_HANG_POWER = 0.8;
    public static double PIVOT_HANG_POWER_LEVEL_TWO = 1.0;
    public static double PIVOT_LEVEL_THREE_POWER = -0.9;
    public static double PIVOT_LEVEL_THREE_TIP_POWER = -0.8;
    public static double PIVOT_PIVOT_DOWN_POWER = -1;
    public static int PIVOT_FIRST_HANG = 434;
    public static int PIVOT_UP_THRESHOLD = 590; //900;
    public static int PIVOT_TOLERANCE = 20;//15;
    public static int PIVOT_AUTO_TOLERANCE = 500; //450; //200;
    public static int PIVOT_HANG_READY_POSITION = 740;
    public static double PIVOT_DOWN_FIRST_POWER = 0.5;
    public static double PIVOT_POWER_HOLD = 0.3;//0.3; //0.2;
    public static double SLIDE_POWER_TEST = 0.1;
    public static double SLIDE_POWER = 0.9;//0.2;nex
    public static double SLIDE_POWER_HANG_LEVEL_TWO = 0.9;//1.0;
    public static double SLIDE_FIRST_HANG_POWER = 0.65;
    public static double SLIDE_POWER_RELEASE = 0.1;
    public static double SLIDE_HOLD_POWER = 0.15;
    public static double SLIDE_POWER_HANG_HOLD = 0.9; //1.0; //0.6;
    public static double SLIDE_POWER_HANG_NEGATIVE_HOLD = -1.0;
    public static int SLIDE_SLOW_STEP = 20;
    public static int SLIDE_STEP = 240;//60;
    public static int SLIDE_HANG_NEGATIVE_TICKS = -1700;
    public static int SLIDE_LEFT_HORIZ_LEGAL_LIMIT_TICKS = 976;//1356; //1240; //844; //967; //1033;//1444; //1372; //1472;//2027;
    public static int SLIDE_RIGHT_HORIZ_LEGAL_LIMIT_TICKS = 976;//1356; //844; //967; //1033; //1444; //1372; //1472;//2027;
    public static int SLIDE_LEFT_HEIGHT_SCORE_TICKS = 2000;//2001;//2560;//2000;//1807;//1936; //3568;
    public static int SLIDE_RIGHT_HEIGHT_SCORE_TICKS = 2000;//2001;//2560;//2000;//1807;//1936; //3569;
    public static int SLIDE_LEFT_MAX = 2250;//1968;//3139;//2319;//3962;
    public static int SLIDE_RIGHT_MAX = SLIDE_LEFT_MAX;//3967;
    public static int SLIDE_LEFT_HANG_HEIGHT = 50; //410; //810;//1450;old hang for 312 motor
    public static int SLIDE_RIGHT_HANG_HEIGHT = 50; //410; //810;//1450;old hang for 312 motor
    public static int SLIDE_HANG_RELEASE_TICKS = 2650;
    public static int SLIDE_HANG_DOWN_TO_BAR_TICKS = 300;
    public static int SLIDE_HOVER_L2_TICKS = 1800;//1878;//1908; //1878;
    public static int FIRST_SAMPLE_SLIDE_TICKS = 650;//1340;
    public static int SECOND_SAMPLE_SLIDE_TICKS = 1340;
    public static int THIRD_SAMPLE_SLIDE_TICKS = 1690;
    public static int SLIDE_LEFT_FIRST_HANG_HEIGHT = 351;//369;
    public static int SLIDE_RIGHT_FIRST_HANG_HEIGHT = 351;
    public static int SLIDE_LEFT_MIN_LIMIT_BUFFER = 50; //10;
    public static int SLIDE_RETRACT_FIRST_POSITION = 49;
    public static int SLIDE_BACK_MIN_LIMIT_BUFFER=10;
    public static int SLIDE_FRONT_STOP_TICKS = SLIDE_LEFT_HEIGHT_SCORE_TICKS;
    public static int SLIDE_BACK_STOP_TICKS = SLIDE_RIGHT_HEIGHT_SCORE_TICKS; //3967;
    public static int SLIDE_NO_DOWN_TICKS = 1300;
    public static int SLIDE_TOLERANCE = 50;//100;//150;
    public static int SLIDE_AUTO_TOLERANCE = SLIDE_TOLERANCE;
    public static int SLIDE_AUTO_RETRACT_TICKS = 800;
    public static int SLIDE_AUTO_RETRACT_3_SAMPLE_TICKS = 500;
    public static double LEFT_DERAIL_UP_SECOND = 0.6;
    public static double RIGHT_DERAIL_UP_SECOND = 0.4;
    public static double LEFT_DERAIL_UP_FIRST = 0.8;
    public static double RIGHT_DERAIL_UP_FIRST = 0.2;
    public static double LEFT_DERAIL_DOWN = 0.0;
    public static double RIGHT_DERAIL_DOWN = 1.0;
    public static double TENSION_POWER = 1.0;
    public static int TENSION_RUN_TIME = 8000; //5000;//500;
    public static double INTAKE_DISTANCE_STOP_CM = 7.35;
    public static double WRIST_STOW_POS = 1.0; //0.89; //0.0; old servo
    public static double WRIST_READY_POS = 0.32; //0.4; //0.33; //0.12; old servo
    public static double WRIST_GATHER_POS = 0.18; // 0.15;
    public static double WRIST_SCORE_POS = 0.6; //0.5; //1; // 0.7 0.5
    public static double AUTO_INTAKE_THRESHOLD = 640.0;//700 //600.0;//800.0;
    public static double AUTO_INTAKE_PICKUP_THRESHOLD = 450;//300.0; //650.0; //700.0; //640.0; //600.0; //550.0; //600.0;
    public static double AUTO_FIRST_INTAKE_PICKUP_THRESHOLD = 800.0;//500.0; TODO: if color sensor broken, use 500
    public static double AUTO_INTAKE_DIAG_PICKUP_THRESHOLD = 800.0;//1200.0; //900.0; TODO: if color sensor broken, use 900
    public static double INTAKE_LEFT_POWER = -0.8;
    public static double INTAKE_RIGHT_POWER = -0.8;
    public static double INTAKE_LEFT_POWER_SCORE = 0.8;
    public static double INTAKE_RIGHT_POWER_SCORE = 0.8;
    private static double SLIDE_POWER_SCORE_HOLD = 0.3;//0.45;
    private static double SLIDE_POWER_EXTEND_RAW_DRIVE = 0.1;
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
    public static double SLIDE_RESET_POWER = -0.8;
    private boolean hasSlidDown = false;
    private CommandScheduler scheduler = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    private RevColorSensorV3 intakeColor = null;
    private ServoImplEx wrist = null;
    public static double AUTO_DETECT_TIME = 40.0;
    public static double AUTO_DETECT_TIME_SAMPLE = 40.0;




    public SlideArm(HardwareMap hm, Telemetry tele, boolean resetMotors) {
        scheduler = CommandScheduler.getInstance();
        hardwareMap = hm;
        telemetry = tele;
        telemetry.addData("Slide arm status", "initializing");

        pivotMotor = hardwareMap.get(DcMotor.class,"pivot_motor");
        //pivotMotorEncoder = hardwareMap.get();
        //pivotEncoder = hardwareMap.get(DcMotor.class, "right_front");
        leftSlideMotor = hardwareMap.get(DcMotor.class,"left_slide_motor" );
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_slide_motor");
        leftIntake = hardwareMap.get(CRServo.class, "intake_left");
        rightIntake = hardwareMap.get(CRServo.class, "intake_right");
        intakeColor = hardwareMap.get(RevColorSensorV3.class, "intake_color");
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
        rightDerailer = hardwareMap.get(ServoImplEx.class, "right_derailer");
        leftDerailer = hardwareMap.get(ServoImplEx.class, "left_derailer");
        rightTensioner = hardwareMap.get(CRServo.class, "right_tensioner");
        leftTensioner = hardwareMap.get(CRServo.class, "left_tensioner");
        hasSlidDown = false;


        if (resetMotors){
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //pivotEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        //pivotMotor.setTargetPosition(0);
        //leftSlideMotor.setTargetPosition(0);
        //rightSlideMotor.setTargetPosition(0);

        //pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //TODO: set polarity of motors
        //pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        autoIntakeStarted = false;
        sleepStarted = false;
        pivotUpRequested = false;
        pivotDownRequested = false;
        pivotIsResting = true;
        hangRequested = false;
        unzeroed = false;
        hasDerailed = false;
        firstHangRequested = false;
        doRelease = false;
        isHolding = false;
        tensionDone = false;
        tensionStarted = false;
        tensionDownStarted = false;
        isPivotingDownAndRetracting = false;
        tensionRightSlide = true;
        loosenRightSlide = true;
        hasSlidDown = false;
        tensioningDone = false;

        telemetry.addData("Slide arm status", "init complete");
    }

    public void update(){
        if (pivotUpRequested && pivotMotor.getCurrentPosition() +PIVOT_TOLERANCE > PIVOT_UP_POSITION ) {
            telemetry.addData("PivotUp", "Completed");
            pivotUpRequested = false;
            pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pivotMotor.setPower(PIVOT_POWER_HOLD);
        }
        if (pivotDownRequested && pivotMotor.getCurrentPosition() - PIVOT_TOLERANCE < PIVOT_DOWN_POSITION) { //&& pivotEncoder.getCurrentPosition() -PIVOT_THROUGHBORE_THRESHOLD < PIVOT_THROUGHBORE_DOWN_POSITION ) {
            telemetry.addData("PivotDown", "Completed");
            pivotDownRequested = false;
            pivotIsResting = true;
            pivotMotor.setPower(0);
        }
        if (isHolding
                && pivotPosition == PivotState.DOWN && pivotMotor.getCurrentPosition() -PIVOT_TOLERANCE < PIVOT_DOWN_POSITION
                 ){
            leftSlideMotor.setPower(0);
            rightSlideMotor.setPower(0);
            isHolding = false;
        }
        if (doRelease){
            int LIMIT = SLIDE_LEFT_MAX;
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
    public void zeroMotors(){
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //pivotEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }




    public void pivotForTallHang(){
        //firstHangRequested = true;
        hangRequested = true;
        if (pivotIsResting){
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        pivotIsResting = false;
        pivotMotor.setTargetPosition(PIVOT_TALL_HANG_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(PIVOT_HANG_POWER);
        pivotPosition = PivotState.UP;
    }
    public void pivotDownForTallHang(){
        hangRequested = true;
        pivotIsResting = false;
        //pivotMotor.setTargetPosition(PIVOT_HANG_PULL_UP_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotPosition = PivotState.UP;
    }






    public void pivotHang(){
        //firstHangRequested = true;
        hangRequested = true;
        if (pivotIsResting){
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        pivotIsResting = false;
        pivotMotor.setTargetPosition(PIVOT_HANG_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(PIVOT_HANG_POWER);
        pivotPosition = PivotState.UP;
    }
    public void pivotLevelTwoHang(){
        hangRequested = true;
        if (pivotIsResting){
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        pivotIsResting = false;
        pivotMotor.setTargetPosition(PIVOT_HANG_LEVEL_TWO_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(PIVOT_HANG_POWER_LEVEL_TWO);
        pivotPosition = PivotState.UP;
    }
    public void pivotLevelTwoTip(){
        pivotIsResting = false;
        pivotMotor.setTargetPosition(PIVOT_HANG_LEVEL_TWO_TIP);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(PIVOT_HANG_POWER_LEVEL_TWO);
        pivotPosition = PivotState.UP;
    }
    public void pivotLevelTwoHook(){
        pivotIsResting = false;
        pivotMotor.setTargetPosition(PIVOT_HANG_LEVEL_TWO_HOOK);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(PIVOT_HANG_POWER_LEVEL_TWO);
        pivotPosition = PivotState.UP;
    }
    public void pivotDownThirdLevelHang(){
        pivotIsResting = false;
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(PIVOT_PIVOT_DOWN_POWER);
        pivotPosition = PivotState.UP;
    }
    public boolean pivotIsBusy() {
        return pivotMotor.isBusy();
    }
    public boolean slidesAreBusy() {
        return leftSlideMotor.isBusy() || rightSlideMotor.isBusy();
    }
    public void nesetPivotHang(){
        //firstHangRequested = true;
        hangRequested = true;
        if (pivotIsResting){
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        pivotIsResting = false;
        pivotMotor.setTargetPosition(PIVOT_HANG_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(PIVOT_HANG_POWER);
    }

    public void pivotDownRetract(){

            pivotMotor.setTargetPosition(PIVOT_DOWN_FIRST_POSITION);
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pivotMotor.setPower(PIVOT_DOWN_FIRST_POWER);
            leftSlideMotor.setTargetPosition(SLIDE_RETRACT_FIRST_POSITION);
            rightSlideMotor.setTargetPosition(SLIDE_RETRACT_FIRST_POSITION);
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftSlideMotor.setPower(SLIDE_FIRST_HANG_POWER);

    }

    public void hangHoldForTallHang() {
        if (!hangRequested) {
            return;
        }
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setTargetPosition(SLIDE_HANG_NEGATIVE_TICKS);
        rightSlideMotor.setTargetPosition(SLIDE_HANG_NEGATIVE_TICKS);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(SLIDE_POWER_HANG_NEGATIVE_HOLD);
        rightSlideMotor.setPower(SLIDE_POWER_HANG_NEGATIVE_HOLD);
    }

    public void hangHold(){
        if (!hangRequested){
            return;
        }
        /*if (pivotMotor.getCurrentPosition() >= PIVOT_DOWN_WHILE_HANG){
            pivotMotor.setTargetPosition(PIVOT_DOWN_WHILE_HANG);
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pivotMotor.setPower(PIVOT_HANG_POWER);
        }

        leftSlideMotor.setTargetPosition(SLIDE_LEFT_HANG_HEIGHT);
        rightSlideMotor.setTargetPosition(SLIDE_RIGHT_HANG_HEIGHT);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(SLIDE_POWER_HANG_HOLD);
        rightSlideMotor.setPower(SLIDE_POWER_HANG_HOLD);
        */
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(0);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setTargetPosition(SLIDE_HANG_NEGATIVE_TICKS);
        rightSlideMotor.setTargetPosition(SLIDE_HANG_NEGATIVE_TICKS);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(SLIDE_POWER_HANG_NEGATIVE_HOLD);
        rightSlideMotor.setPower(SLIDE_POWER_HANG_NEGATIVE_HOLD);

        //pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //pivotMotor.setTargetPosition(PIVOT_DOWN_FOR_HANG);
        //pivotMotor.setPower(0.8);
        /*
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setPower(-0.8);
        rightSlideMotor.setPower(0.8);
        */
    }
    public void sit(){
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(1.0);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void stopHang() {
        hangRequested = false;
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setPower(0);
        leftSlideMotor.setPower(0);
    }

    public void firstHangHold(){
        if (!firstHangRequested){
            return;
        }
        leftSlideMotor.setTargetPosition(SLIDE_LEFT_FIRST_HANG_HEIGHT);
        rightSlideMotor.setTargetPosition(SLIDE_RIGHT_FIRST_HANG_HEIGHT);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(SLIDE_POWER_HANG_HOLD);
        rightSlideMotor.setPower(SLIDE_POWER_HANG_HOLD);
    }


    public void hangRelease(){
        if (!hangRequested) {// && !firstHangRequested){
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
                if (pivotIsResting){
                    pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                pivotIsResting = false;
                pivotPosition = PivotState.UP;
                pivotMotor.setTargetPosition(PIVOT_UP_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_UP);
                pivotUpRequested = true;
//                break;
//            default:
//                telemetry.addData("Problem", "Pivot Up Invalid Position");
//        }
    }
    public void changePower(double power){
        PIVOT_POWER_UP = power;
        PIVOT_POWER_LIFT_BODY = 1.0;
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
        if(getFrontSlideTicks()> SLIDE_NO_DOWN_TICKS){
            return;
        }
                hangRequested = false;
                pivotPosition = PivotState.DOWN;
                pivotMotor.setTargetPosition(PIVOT_DOWN_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_DOWN);
                pivotDownRequested = true;
//                break;
//            default:
//                telemetry.addData("Problem", "Pivot Down Invalid Position");
//        }
    }
    public void manualPivotDown(){
        hangRequested = false;
        pivotPosition = PivotState.MANUAL;
        pivotMotor.setTargetPosition(PIVOT_DOWN_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(PIVOT_POWER_LIFT_BODY);
    }
    public void hangPivotUp(){
        pivotPosition = null;
        //hangRequested = true;
        if (pivotIsResting){
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        switch (pivotHangPosition) {
            case UP:
                hangRequested = false;
                break;
            case READY:
                hangRequested = false;
                pivotHangPosition = PivotHangState.UP;
                pivotMotor.setTargetPosition(PIVOT_UP_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_UP);
                pivotUpRequested = true;
                break;
            case BAR:
                hangRequested = false;
                pivotHangPosition = PivotHangState.READY;
                pivotMotor.setTargetPosition(PIVOT_HANG_READY_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_UP);
                pivotUpRequested = true;
                break;
            case DOWN:
                hangRequested = false;
                pivotHangPosition = PivotHangState.READY;
                pivotMotor.setTargetPosition(PIVOT_HANG_READY_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_UP);
                pivotUpRequested = true;
                break;
            case HANG:
                hangRequested = true;
                break;
        }

    }
    public void hangPivotDown(){
        pivotPosition = null;
        //hangRequested = true;
        if (pivotIsResting){
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        switch (pivotHangPosition) {
            case UP:
                hangRequested = false;
                pivotHangPosition = PivotHangState.READY;
                pivotMotor.setTargetPosition(PIVOT_HANG_READY_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_DOWN);
                pivotDownRequested = true;
                break;
            case READY:
                hangRequested = false;
                pivotHangPosition = PivotHangState.BAR;
                pivotMotor.setTargetPosition(PIVOT_HANG_BAR_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_DOWN);
                pivotDownRequested = true;
                break;
            case BAR:
                hangRequested = false;
                pivotHangPosition = PivotHangState.DOWN;
                pivotMotor.setTargetPosition(PIVOT_DOWN_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_DOWN);
                pivotDownRequested = true;
                break;
            case DOWN:
                hangRequested = false;
                break;
            case HANG:
                hangRequested = true;
                break;
        }

    }
    public void derailShiftDown() {
        rightDerailer.setPosition(RIGHT_DERAIL_DOWN);
        leftDerailer.setPosition(LEFT_DERAIL_DOWN);
    }
    public void derailRelax() {
        rightDerailer.setPwmDisable();
        leftDerailer.setPwmDisable();
    }
    public void derailShiftFirst() {
        rightDerailer.setPosition(RIGHT_DERAIL_UP_FIRST);
        leftDerailer.setPosition(LEFT_DERAIL_UP_FIRST);
    }
    public void derailShiftSecond() {
        rightDerailer.setPosition(RIGHT_DERAIL_UP_SECOND);
        leftDerailer.setPosition(LEFT_DERAIL_UP_SECOND);
    }
    public void stopTensioning() {
        rightTensioner.setPower(0);
        leftTensioner.setPower(0);
        tensionStarted = false;
        tensionDownStarted = false;
    }
    public void tensionSlidesManual() {
        if(!tensionStarted) {
            tensionStarted=true;
            rightTensioner.setPower(TENSION_POWER);
            leftTensioner.setPower(TENSION_POWER);
        }
    }
    public void tensionSlidesIndividual() {
        if(!tensionStarted) {
            tensionStarted=true;
            if(tensionRightSlide) {
                rightTensioner.setPower(TENSION_POWER);
            } else {
                leftTensioner.setPower(TENSION_POWER);
            }
        }
    }
    public boolean tensionSlides() {
        if(!tensionStarted) {
            tensionDone = false;
            tensionTime.reset();
            tensionStarted = true;
            rightTensioner.setPower(TENSION_POWER);
            leftTensioner.setPower(TENSION_POWER);
        }
        else if (tensionStarted && tensionTime.milliseconds() > TENSION_RUN_TIME){
            rightTensioner.setPower(0);
            leftTensioner.setPower(0);
            tensionStarted = false;
            tensionDone = true;
            return !tensionStarted;
        }
        return !tensionStarted;
    }
    public void loosenSlidesManual() {
        if(!tensionDownStarted) {
            tensionDownStarted=true;
            rightTensioner.setPower(-TENSION_POWER);
            leftTensioner.setPower(-TENSION_POWER);
        }
    }
    public void loosenSlidesIndividual() {
        if(!tensionStarted) {
            tensionStarted=true;
            if(loosenRightSlide) {
                rightTensioner.setPower(-TENSION_POWER);
            } else {
                leftTensioner.setPower(-TENSION_POWER);
            }
        }
    }
    public void scoreAuto() {
        if (!isPivotUp()) {
            pivotUp();
            autoPivotTime.reset();
        }
        if (pivotMotor.getCurrentPosition() >= PIVOT_AUTO_TOLERANCE){
            slideUpOneStep();
        }

        if (isSlideUpFull() && !isWristScore()){
            wrist.setPosition(WRIST_SCORE_POS);
        }
    }
    public boolean isWristScore(){
        return wristPosition == WristState.SCORE;
    }
    public boolean isPivotUp(){
        return pivotPosition == PivotState.UP;
    }
    public boolean isSlideUpFull(){
        return leftSlideMotor.getCurrentPosition() >= SLIDE_LEFT_HEIGHT_SCORE_TICKS-SLIDE_TOLERANCE;
    }

    public void holdPivot(){
        pivotIsResting = false;
        //pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition() + PIVOT_HANG_LEVEL_THREE_HOLD_MOVE);
        //pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(PIVOT_LEVEL_THREE_POWER);
    }

    public void firstPivot(){
        if (pivotIsResting){
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        pivotIsResting = false;
        pivotMotor.setTargetPosition(PIVOT_FIRST_HANG);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(PIVOT_HANG_POWER);
    }

    public void makeLimp() {
        pivotMotor.setPower(0);
        rightSlideMotor.setPower(0);
        leftSlideMotor.setPower(0);
        wrist.setPwmDisable();
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
    public void makePivotLimp() {
        pivotMotor.setPower(0);
    }
    public void makeHangPivotLimp() {
        pivotMotor.setPower(0);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public int getPivotTicks(){
        return pivotMotor.getCurrentPosition();
    }
    public PivotState getPivotState() {
        return pivotPosition;
    }
    public WristState getWristPosition(){
        return wristPosition;
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
        if (pivotPosition == PivotState.DOWN && pivotMotor.getCurrentPosition() -PIVOT_TOLERANCE < PIVOT_DOWN_POSITION)
                {
            leftSlideMotor.setPower(0);
            rightSlideMotor.setPower(0);
            isHolding = false;
        }else {
            leftSlideMotor.setTargetPosition(leftSlideMotor.getCurrentPosition());
            rightSlideMotor.setTargetPosition(rightSlideMotor.getCurrentPosition());
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
            rightSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
            isHolding = true;
        }

    }
    public void slideUpRawDrive(){
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftSlideMotor.setPower(SLIDE_POWER_EXTEND_RAW_DRIVE);
            rightSlideMotor.setPower(SLIDE_POWER_EXTEND_RAW_DRIVE);
    }
    public void releaseSlides(){
        leftSlideMotor.setPower(0);
        rightSlideMotor.setPower(0);
        isHolding = false;
    }
    public void slideUpHoverLevel2(){
        leftSlideMotor.setTargetPosition(SLIDE_HOVER_L2_TICKS);
        rightSlideMotor.setTargetPosition(SLIDE_HOVER_L2_TICKS);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(SLIDE_POWER);
        rightSlideMotor.setPower(SLIDE_POWER);
        isHolding = true;
    }
    public void slideDownToBar(){
        leftSlideMotor.setTargetPosition(leftSlideMotor.getCurrentPosition() + SLIDE_HANG_DOWN_TO_BAR_TICKS);
        rightSlideMotor.setTargetPosition(rightSlideMotor.getCurrentPosition() + SLIDE_HANG_DOWN_TO_BAR_TICKS);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
        rightSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
        isHolding = true;
    }
    public boolean slideUpOneStep() {
        int LIMIT = SLIDE_LEFT_HORIZ_LEGAL_LIMIT_TICKS;
        boolean isLeftDone = false;
        boolean isRightDone = false;
        if (pivotMotor.getCurrentPosition() > PIVOT_UP_THRESHOLD) {
            LIMIT = SLIDE_LEFT_HEIGHT_SCORE_TICKS;
        }
        if (hangRequested) {
            LIMIT = SLIDE_LEFT_MAX;
        }

        if (leftSlideMotor.getCurrentPosition() <= LIMIT - SLIDE_TOLERANCE) {
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlideMotor.setPower(SLIDE_POWER);
        } else {
            leftSlideMotor.setTargetPosition(LIMIT); //leftSlideMotor.getCurrentPosition());
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
            isLeftDone = true;
        }
        if (rightSlideMotor.getCurrentPosition() <= LIMIT - SLIDE_TOLERANCE) {
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setPower(SLIDE_POWER);
        } else {
            rightSlideMotor.setTargetPosition(LIMIT); //rightSlideMotor.getCurrentPosition());
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
            isRightDone = true;
        }
        return !isLeftDone || !isRightDone;
    }
        public boolean slideToSampleOne() {
            int SAMPLE_LIMIT = FIRST_SAMPLE_SLIDE_TICKS;
            boolean isLeftDone = false;
            boolean isRightDone   = false;
            if (pivotMotor.getCurrentPosition() > PIVOT_UP_THRESHOLD) {
                SAMPLE_LIMIT = SLIDE_LEFT_HEIGHT_SCORE_TICKS;
            }


            if (leftSlideMotor.getCurrentPosition() <= SAMPLE_LIMIT - SLIDE_TOLERANCE) {
                leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftSlideMotor.setPower(SLIDE_POWER);
            } else {
                leftSlideMotor.setTargetPosition(SAMPLE_LIMIT); //leftSlideMotor.getCurrentPosition());
                leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
                isLeftDone = true;
            }
            if (rightSlideMotor.getCurrentPosition() <= SAMPLE_LIMIT - SLIDE_TOLERANCE) {
                rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightSlideMotor.setPower(SLIDE_POWER);
            } else {
                rightSlideMotor.setTargetPosition(SAMPLE_LIMIT); //rightSlideMotor.getCurrentPosition());
                rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
                isRightDone = true;
            }
            return !isLeftDone || !isRightDone;
        }
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


    public boolean slideUpOneStepWithoutLimits(){
        int LIMIT = SLIDE_LEFT_MAX;
        boolean isLeftDone = false;
        boolean isRightDone = false;

        if(leftSlideMotor.getCurrentPosition() <= LIMIT - SLIDE_TOLERANCE){
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlideMotor.setPower(SLIDE_POWER);
        }else{
            leftSlideMotor.setTargetPosition(LIMIT); //leftSlideMotor.getCurrentPosition());
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
            isLeftDone = true;
        }
        if(rightSlideMotor.getCurrentPosition() <= LIMIT - SLIDE_TOLERANCE) {
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setPower(SLIDE_POWER);
        }else{
            rightSlideMotor.setTargetPosition(LIMIT); //rightSlideMotor.getCurrentPosition());
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setPower(SLIDE_POWER_SCORE_HOLD);
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
    public void slideUpOneStepWithoutLimitsHang() {

        boolean isLeftDone = false;
        boolean isRightDone = false;

        leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setPower(SLIDE_POWER);

        rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setPower(SLIDE_POWER);
    }
    public void slideDownOneStepWithoutLimitsHang() {
        boolean isLeftDone = false;
        boolean isRightDone = false;

            leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlideMotor.setPower(-SLIDE_POWER_HANG_LEVEL_TWO);

            rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setPower(-SLIDE_POWER_HANG_LEVEL_TWO);
    }
    public void slideDownFullPower(){
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setPower(-1.0);

        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setPower(-1.0);
    }
    public void slideUpSmallPower(){
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setPower(-0.3);

        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setPower(-0.3);
    }
    public boolean slideDownOneStepAuto() {
            doRelease = false;
            int LIMIT = SLIDE_AUTO_RETRACT_TICKS;
            boolean isLeftDone = false;
            boolean isRightDone = false;
            if (leftSlideMotor.getCurrentPosition() >= LIMIT + SLIDE_AUTO_TOLERANCE) {
                leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftSlideMotor.setPower(-SLIDE_POWER);
            } else {
                leftSlideMotor.setPower(0);
                isLeftDone = true;
            }
            if (rightSlideMotor.getCurrentPosition() >= LIMIT + SLIDE_AUTO_TOLERANCE) {
                rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightSlideMotor.setPower(-SLIDE_POWER);
            } else {
                rightSlideMotor.setPower(0);
                isRightDone = true;
            }
            return !isLeftDone || !isRightDone;
        }
    public boolean slideDownDerail() {
        if (!hasSlidDown) {
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlideMotor.setTargetPosition(leftSlideMotor.getCurrentPosition() - 800);
            rightSlideMotor.setTargetPosition(rightSlideMotor.getCurrentPosition() - 800);
            leftSlideMotor.setPower(SLIDE_POWER);
            rightSlideMotor.setPower(SLIDE_POWER);
            hasSlidDown = true;
        }
        return slidesAreBusy();
    }
    public boolean slideDownOneStepTo3Sample() {
        doRelease = false;
        int LIMIT = SLIDE_AUTO_RETRACT_3_SAMPLE_TICKS;
        boolean isLeftDone = false;
        boolean isRightDone = false;
        if (leftSlideMotor.getCurrentPosition() >= LIMIT + SLIDE_AUTO_TOLERANCE) {
            leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlideMotor.setPower(-SLIDE_POWER);
        } else {
            leftSlideMotor.setPower(0);
            isLeftDone = true;
        }
        if (rightSlideMotor.getCurrentPosition() >= LIMIT + SLIDE_AUTO_TOLERANCE) {
            rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideMotor.setPower(-SLIDE_POWER);
        } else {
            rightSlideMotor.setPower(0);
            isRightDone = true;
        }
        return !isLeftDone || !isRightDone;
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

    public void stopPivoting() {
        pivotMotor.setPower(0);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void pivotBrakes() {
        pivotMotor.setPower(0);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public int getFrontSlideTicks() {
        return leftSlideMotor.getCurrentPosition();
    }
    public int getBackSlideTicks() {
        return rightSlideMotor.getCurrentPosition();
    }
    public void slideBrakes() {
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setPower(-SLIDE_HOLD_POWER);
        rightSlideMotor.setPower(SLIDE_HOLD_POWER);


    }
    public boolean activateIntakeWithSensor(){
        boolean isDone = false;
        if (intakeColor.getDistance(DistanceUnit.CM) > INTAKE_DISTANCE_STOP_CM){
            if (!isWristGather()){
                setWristToGather();

            }
            leftIntake.setPower(INTAKE_LEFT_POWER);
            rightIntake.setPower(INTAKE_RIGHT_POWER);
            servoStatus = "Intake with Sensor Active";
            intakeTime.reset();
        } else {
            if (intakeTime.milliseconds() > AUTO_DETECT_TIME ){ //&& isKeeperBlock()) {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                servoStatus = "Inactive";
                if (isWristGather()){
                    setWristToScore();
                }
                isDone = true;
                return isDone;
                //slideDownOneStepAuto();
            } else {
                servoStatus = "Intake sensed object and waiting";
            }
        }
        return isDone;
    }
    public boolean activateIntakeWithSensorSampleAuto(){
        boolean isDone = false;
        if (intakeColor.getDistance(DistanceUnit.CM) > INTAKE_DISTANCE_STOP_CM){
            if (!isWristGather()){
                setWristToGather();

            }
            leftIntake.setPower(INTAKE_LEFT_POWER);
            rightIntake.setPower(INTAKE_RIGHT_POWER);
            servoStatus = "Intake with Sensor Active";
            intakeTime.reset();
        } else {
            if (intakeTime.milliseconds() > AUTO_DETECT_TIME_SAMPLE ){ //&& isKeeperBlock()) {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                servoStatus = "Inactive";
                if (isWristGather()){
                    setWristToScore();
                }
                isDone = true;
                return isDone;
                //slideDownOneStepAuto();
            } else {
                servoStatus = "Intake sensed object and waiting";
            }
        }
        return isDone;
    }

    public SampleColor detectColor(){
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
    }


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
    public void setWristToGather(){
        wrist.setPosition(WRIST_GATHER_POS);
        wristPosition = WristState.GATHER;
    }
    public void setWristToScore(){
        wrist.setPosition(WRIST_SCORE_POS);
        wristPosition = WristState.SCORE;
    }
    public boolean isWristGather(){
        return wristPosition == WristState.GATHER;
    }

    public void nextWristPosition(){
        switch(wristPosition) {
            /* case STOW:
                wrist.setPosition(WRIST_STOW_POS);
                wristPosition = WristState.READY;
                break;*/
            case GATHER:
                wrist.setPosition(WRIST_SCORE_POS);
                wristPosition = WristState.SCORE;
                break;
            case SCORE:
                wrist.setPosition(WRIST_READY_POS);
                wristPosition = WristState.READY;
                break;
            case READY:
            default:
                wrist.setPosition(WRIST_GATHER_POS);
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

    public double getIntakeDistaceCM(){
        return intakeColor.getDistance(DistanceUnit.CM);
    }
    public double getLuminancePb() {
        return LUMINANCE_PB*(intakeColor.blue())-(intakeColor.red()*LUMINANCE_RED+
                LUMINANCE_GREEN*intakeColor.green()+LUMINANCE_BLUE*intakeColor.blue());
    }
    public double getLuminancePr() {
        return LUMINANCE_PR*(intakeColor.red())-(intakeColor.red()*LUMINANCE_RED+
                LUMINANCE_GREEN*intakeColor.green()+LUMINANCE_BLUE*intakeColor.blue());
    }

    public double getPivotPosition(){
        return 0;//pivotEncoder.getCurrentPosition();
    }

    public void addSlideTelemetry(){
        telemetry.addData("Left Slide Ticks", leftSlideMotor.getCurrentPosition());
        telemetry.addData("Right Slide Ticks", rightSlideMotor.getCurrentPosition());
        telemetry.addData("Pivot Ticks", pivotMotor.getCurrentPosition());
        //telemetry.addData("Throughbore Pivot Ticks", pivotEncoder.getCurrentPosition());
        telemetry.addData("PivotUpRequested", pivotUpRequested);
        telemetry.addData("PivotDownRequested", pivotDownRequested);
        telemetry.addData("pivot hang pos", pivotHangPosition);
        telemetry.addData("pivot pos", pivotPosition);
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
    public class DerailShiftFirst extends CommandBase {
        boolean isDone = false;
        public DerailShiftFirst() {}

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            telemetry.addData("Derail", "First");
            telemetry.update();
            derailShiftFirst();
            isDone = true;
            hasDerailed = true;
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }
    public class DerailShiftSecond extends CommandBase {
        boolean isDone = false;
        public DerailShiftSecond() {}

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            telemetry.addData("Derail", "Second");
            telemetry.update();
            derailShiftSecond();
            isDone = true;
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }
    public class DerailExtendSlides extends CommandBase {
        public DerailExtendSlides() {}

        @Override
        public void initialize() {}

        @Override
        public void execute() {
            telemetry.addData("Slides", "Extending");
            telemetry.update();
        }

        @Override
        public boolean isFinished() {
            return !slideUpOneStep();
        }
    }
    public class DerailPivotUp extends CommandBase {
        boolean isDone =  false;
        boolean isFirstTime = true;
        public DerailPivotUp() {}

        @Override
        public void initialize() {
            isDone = false;
            isFirstTime = true;
        }

        @Override
        public void execute() {
            telemetry.addData("Pivoting", "Up");
            telemetry.update();
            if (isFirstTime) {
                isFirstTime = false;
                pivotUp();
            }
        }

        @Override
        public boolean isFinished() {
            return (pivotMotor.getCurrentPosition() + PIVOT_TOLERANCE > PIVOT_UP_POSITION);
        }
    }
    public class DerailRetractSlides extends CommandBase {
        boolean isDone = false;
        public DerailRetractSlides() {}

        @Override
        public void initialize() {
            isDone = false;
            hasSlidDown = false;
        }

        @Override
        public void execute() {
            telemetry.addData("Slide", "Retracting");
            telemetry.update();
            isDone = !slideDownDerail();

        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }
    public class DerailTensionSlides extends CommandBase {
        boolean isDone = false;
        public DerailTensionSlides() {}

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            telemetry.addData("Tensioning", "Slides");
            telemetry.update();
            isDone = tensionSlides();
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }
    public boolean hasDerailed() {
        return hasDerailed;
    }
    public boolean isTensionDone() {
        return tensionDone;
    }

    public void derailResetScheduler() {
        hasDerailed = false;
        scheduler.schedule(
                new SequentialCommandGroup(
                        //this.new DerailPivotUp(),
                        this.new DerailExtendSlides(),
                        this.new DerailShiftFirst()
                )
        );
    }
    public void tensionResetScheduler() {
        hasDerailed = false;
        hasSlidDown = false;
        scheduler.schedule(
                new SequentialCommandGroup(
                        this.new DerailShiftSecond(),
                        new ParallelCommandGroup(
                            this.new DerailTensionSlides(),
                            this.new DerailRetractSlides()
                        )
                )
        );
    }
    public void runSchedule() {
        scheduler.run();
    }







    public class RetractSlideArm extends CommandBase {
        boolean isDone = false;
        public RetractSlideArm() {}

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            telemetry.addData("Slide", "Retracting");
            telemetry.update();
            isDone = !slideDownOneStep();
        }
        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class RetractSlideArmPartial extends CommandBase {
        boolean isDone = false;
        public RetractSlideArmPartial() {}

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            telemetry.addData("Slide", "Retracting");
            telemetry.update();
            isDone = !slideDownOneStepAuto();
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }
    public class RetractSlideArmTo3Sample extends CommandBase {
        boolean isDone = false;
        public RetractSlideArmTo3Sample() {}

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            telemetry.addData("Slide", "Retracting");
            telemetry.update();
            isDone = !slideDownOneStepTo3Sample();
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class ExtendSlideArm extends CommandBase {
        boolean isDone = false;
        public ExtendSlideArm() {}

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            //extendSlideArm();
            telemetry.addData("Slide", "Extending");
            telemetry.update();
            isDone = !slideUpOneStep();
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }
    public class ExtendSlideToSample1 extends CommandBase {
        boolean isDone = false;

        public ExtendSlideToSample1() {
        }

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            //extendSlideArm();
            telemetry.addData("Slide", "Extending");
            telemetry.update();
            isDone = !slideToSampleOne();
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }

    }

    public class PivotSlideArmUp extends CommandBase {
        boolean isDone =  false;
        boolean isFirstTime = true;
        public PivotSlideArmUp() {}

        @Override
        public void initialize() {
            isDone = false;
            isFirstTime = true;
        }

        @Override
        public void execute() {
            //pivotSlideArmUp();
            /*
            if (pivotMotor.getCurrentPosition() +PIVOT_TOLERANCE > PIVOT_UP_POSITION) {
                telemetry.addData("PivotUp", "Completed");
                telemetry.update();
                pivotUpRequested = false;
                pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pivotMotor.setPower(PIVOT_POWER_HOLD);
                isDone = false;
            }
            else if (!pivotUpRequested){
                pivotUpRequested = true;
                pivotPosition = PivotState.UP;
                telemetry.addData("PivotUp", "Starting");
                telemetry.update();
                pivotUp();
                isDone = true;
            }
            telemetry.addData("PivotUp", "Busy");
            telemetry.update();*/
            //pivotUp();
            if (isFirstTime) {
                isFirstTime = false;
                pivotUp();
            }
        }

        @Override
        public boolean isFinished() {
            return (pivotMotor.getCurrentPosition() + PIVOT_TOLERANCE > PIVOT_UP_POSITION);
        }
    }


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
    }*/

    public class PivotSlideArmDown extends CommandBase {
        boolean isDone = false;
        boolean isFirstTime = true;
        public PivotSlideArmDown() {}

        @Override
        public void initialize() {
            isDone = false;
            isFirstTime = true;
        }

        @Override
        public void execute() {
            /*
            if (pivotMotor.getCurrentPosition() - PIVOT_TOLERANCE< PIVOT_DOWN_POSITION) {
                telemetry.addData("PivotDown", "Completed");
                telemetry.update();
                pivotDownRequested = false;
                isDone = false;
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
            isDone = true;*/
            if (isFirstTime){
                isFirstTime = false;
                pivotDown();
            }
        }
        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        @Override
        public boolean isFinished() {
            return (pivotMotor.getCurrentPosition() - PIVOT_TOLERANCE< PIVOT_DOWN_POSITION);
        }
    }

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

    /*public class IntakeWithSensor extends CommandBase {
        public IntakeWithSensor() {}

        @Override
        public void initialize() {}

        @Override
        public void execute() {}

        @Override
        public boolean isFinished() {
            return activateIntakeWithSensor();
        }
    }*/
    public class IntakeScore extends CommandBase {
        boolean isDone = false;
        public IntakeScore() {}


        @Override
        public void initialize() {
            isDone = false;
            autoIntakeStarted = false;
        }

        @Override
        public void execute() {
            //reverseIntakeScore();
            if (!autoIntakeStarted) {
                autoIntakeStarted = true;
                autoIntakeTime.reset();
                activateIntakeWithoutSensor();
                //isDone = false;
            }
            else if (autoIntakeTime.milliseconds() < AUTO_INTAKE_THRESHOLD) {
                activateIntakeWithoutSensor();
                //isDone = false;
            }
            else if (autoIntakeTime.milliseconds() > AUTO_INTAKE_THRESHOLD) {
                stopIntake();
                isDone = true;
            }
            //stopIntake();

        }

        @Override
        public boolean isFinished() {
            return (isDone);
        }
    }



    public class stopIntake extends CommandBase {
        boolean isDone = false;
        public stopIntake() {}
        @Override
        public void initialize() {
            autoIntakeStarted = false;
        }

        @Override
        public void execute() {
            stopIntake();
            isDone = true;
        }

        @Override
        public boolean isFinished() {
            return (isDone);
        }
    }



    public class IntakeReverse extends CommandBase {
        boolean isDone = false;
        public IntakeReverse() {}
        @Override
        public void initialize() {
            isDone = false;
            autoIntakeStarted = false;
        }

        @Override
        public void execute() {
            if (!autoIntakeStarted) {
                autoIntakeStarted = true;
                autoIntakeTime.reset();
                reverseIntakeWithoutSensor();
            }
            else if (autoIntakeTime.milliseconds() < AUTO_INTAKE_PICKUP_THRESHOLD) {
                reverseIntakeWithoutSensor();
            }
            else if (autoIntakeTime.milliseconds() > AUTO_INTAKE_PICKUP_THRESHOLD) {
                stopIntake();
                isDone = true;
            }
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class IntakeDiagonalSample extends CommandBase {
        boolean isDone = false;
        public IntakeDiagonalSample() {}

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            if (!autoIntakeStarted) {
                autoIntakeStarted = true;
                autoIntakeTime.reset();
                activateIntakeWithoutSensor();
                //isDone = false;
            }
            else if (autoIntakeTime.milliseconds() < AUTO_INTAKE_DIAG_PICKUP_THRESHOLD) {
                activateIntakeWithoutSensor();
                //isDone = false;
            }
            else if (autoIntakeTime.milliseconds() > AUTO_INTAKE_DIAG_PICKUP_THRESHOLD) {
                stopIntake();
                isDone = true;
            }
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

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

    public class wristReady extends CommandBase {
        boolean isDone = false;
        public wristReady() {}

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            wrist.setPosition(WRIST_READY_POS);
            isDone = true;
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class wristGather extends CommandBase {
        boolean isDone = false;

        public wristGather() {}
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            wrist.setPosition(WRIST_GATHER_POS);
            isDone = true;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class wristScore extends CommandBase {
        boolean isDone = false;

        public wristScore() {}
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            wrist.setPosition(WRIST_SCORE_POS);
            isDone = true;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class wristStow extends CommandBase {
        boolean isDone = false;
        public wristStow() {}
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            wrist.setPosition(WRIST_STOW_POS);
            isDone = true;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class Sleep250Millisec extends CommandBase {
        boolean isDone = false;
        public Sleep250Millisec() {}
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            //reverseIntakeScore();
            if (!sleepStarted) {
                sleepStarted = true;
                sleepTime.reset();
                isDone = false;//return true;
            }
            else if (sleepTime.milliseconds() < 250) {
                isDone = false;//return true;
            }
            sleepStarted = false;
            isDone = true;//return false;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }
    public class Sleep750Millisec extends CommandBase {
        boolean isDone = false;
        public Sleep750Millisec() {}
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            //reverseIntakeScore();
            if (!sleepStarted) {
                sleepStarted = true;
                sleepTime.reset();
                isDone = false;//return true;
            }
            else if (sleepTime.milliseconds() < 750) {
                isDone = false;//return true;
            }
            sleepStarted = false;
            isDone = true;//return false;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class Sleep500Millisec extends CommandBase {
        boolean isDone = false;
        public Sleep500Millisec() {}
        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            //reverseIntakeScore();
            if (!sleepStarted) {
                sleepStarted = true;
                sleepTime.reset();
                isDone = false;//return true;
            }
            else if (sleepTime.milliseconds() < 500) {
                isDone = false;//return true;
            }
            sleepStarted = false;
            isDone = true;//return false;
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }



}
