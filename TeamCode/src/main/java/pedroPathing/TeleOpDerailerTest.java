package pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystem.OpModeTransfer;
import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;

@Disabled
@TeleOp(name="TeleOpDerailerTest")
@Config
public class TeleOpDerailerTest extends LinearOpMode{

    public static double speedMultiplier = 0.5;
    public static double strafeBiasLeft = -0.045;
    public static double strafeBiasRight = -0.033;
    public static double strafeBiasForward = 0.0;//-0.05;
    public static double strafeBiasBackward = 0.0;//-0.05;
    public static double HIGH_SPEED_THRESHOLD = 0.8;
    public static double LOW_SPEED_THRESHOLD = 0.1;
    private SpecimenArm specimenArm = null;
    private SlideArm slideArm = null;
    private DcMotor pivotMotor = null;
    private DcMotor leftSlideMotor = null;
    private DcMotor rightSlideMotor = null;
    private SlideArm.PivotState pivotPosition = SlideArm.PivotState.DOWN;
    private ServoImplEx rightDerailer = null;
    private ServoImplEx leftDerailer = null;
    private CRServo rightTensioner = null;
    private CRServo leftTensioner = null;
    private boolean slideWasRetract = false;
    private boolean slideWasExtend = false;
    private boolean slideWasAutoExtend = false;
    private boolean wasPivoting = false;
    private boolean isHolding = false;
    private boolean pivotUpRequested = false;
    private boolean doRelease = false;
    private boolean hangRequested = false;
    private boolean pivotDownRequested = false;
    private static double SLIDE_POWER_SCORE_HOLD = 0.3;//0.45;
    public static int SLIDE_LEFT_MIN_LIMIT_BUFFER = 50; //10;
    public static int SLIDE_TOLERANCE = 50;
    public static double SLIDE_POWER = 0.9;//0.2;nex
    public static double RIGHT_DERAIL_UP = 0.4;
    public static double LEFT_DERAIL_UP = 0.6;
    public static double RIGHT_DERAIL_DOWN = 1.0;
    public static double LEFT_DERAIL_DOWN = 0.0;
    public static int SLIDE_LEFT_HORIZ_LEGAL_LIMIT_TICKS = 1008; //1240; //844; //967; //1033;//1444; //1372; //1472;//2027;
    public static int PIVOT_UP_THRESHOLD = 590; //900;
    public static int SLIDE_LEFT_HEIGHT_SCORE_TICKS = 1936; //3568;
    public static int SLIDE_LEFT_HANG_LIMIT = 2200;//2319;//3962;
    public static int PIVOT_TOLERANCE = 15;
    public static int PIVOT_DOWN_POSITION = 2;
    public static double PIVOT_POWER_UP = 0.7; //0.6; //0.7;
    public static double PIVOT_POWER_HANG = 1;
    public static int PIVOT_DOWN_HANG_POSITION = 0;//776;//1400; //1500; //1350; //1245; measured on old motor//1031 actual;//335
    public static int PIVOT_HANG_POSITION = 400;
    public static int PIVOT_HANG_UP_POSITION = 740;
    public static int SLIDE_NO_DOWN_TICKS = 1000;
    public static double PIVOT_POWER_DOWN = 0.3; //0.7; //0.05;


    private Follower follower;
    @Override
    public void runOpMode() throws InterruptedException{
        specimenArm = new SpecimenArm(hardwareMap, telemetry, false);
        slideArm = new SlideArm(hardwareMap, telemetry, false);
        pivotMotor = hardwareMap.get(DcMotor.class,"pivot_motor");
        leftSlideMotor = hardwareMap.get(DcMotor.class,"left_slide_motor" );
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_slide_motor");
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(OpModeTransfer.autoPose);

        rightDerailer = hardwareMap.get(ServoImplEx.class, "right_derailer");
        leftDerailer = hardwareMap.get(ServoImplEx.class, "left_derailer");
        rightTensioner = hardwareMap.get(CRServo.class, "right_tensioner");
        leftTensioner = hardwareMap.get(CRServo.class, "left_tensioner");
        rightDerailer.setPwmEnable();
        leftDerailer.setPwmEnable();
        waitForStart();

        slideArm.setWristToStow();
        //follower.startTeleopDrive();
        while (opModeIsActive()) {

            if (gamepad2.dpad_left) {
                slideWasRetract = true;
                //slideArm.slideDownOneStep();
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

            }
            if (slideWasRetract && !gamepad2.dpad_left) {
                slideWasRetract = false;
                //slideArm.stopSliding();
                //leftSlideMotor.setPower(0);
                //rightSlideMotor.setPower(0);
                if (pivotPosition == SlideArm.PivotState.DOWN && pivotMotor.getCurrentPosition() -PIVOT_TOLERANCE < PIVOT_DOWN_POSITION)
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
            if (gamepad2.dpad_right) {
                slideWasExtend = true;
                //slideArm.slideUpOneStep();
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

            }
            if (slideWasExtend && !gamepad2.dpad_right) {
                slideWasExtend = false;
                //slideArm.stopSliding();
                //leftSlideMotor.setPower(0);
                //rightSlideMotor.setPower(0);
                if (pivotPosition == SlideArm.PivotState.DOWN && pivotMotor.getCurrentPosition() -PIVOT_TOLERANCE < PIVOT_DOWN_POSITION)
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

            if (gamepad2.a) {
                rightTensioner.setPower(1);
                leftTensioner.setPower(1);
            }
            if (gamepad2.dpad_up) {
                rightTensioner.setPower(-1);
                leftTensioner.setPower(-1);
            }
            if (gamepad2.b) {
                rightTensioner.setPower(0);
                leftTensioner.setPower(0);
            }
            if (gamepad2.x) { //down
                rightDerailer.setPosition(RIGHT_DERAIL_UP);
                leftDerailer.setPosition(LEFT_DERAIL_UP);
            }
            if (gamepad2.y) {
                rightDerailer.setPosition(RIGHT_DERAIL_DOWN);
                leftDerailer.setPosition(LEFT_DERAIL_DOWN);
            }
            if (gamepad2.dpad_down) {
                rightDerailer.setPosition(0.70);
                leftDerailer.setPosition(0.30);
            }

            /*
            double leftJoyStickSpeedY = gamepad1.left_stick_y;
            double leftJoyStickSpeedX = gamepad1.left_stick_x;
            double strafeBias = strafeBiasLeft;
            double strafeBiasY = strafeBiasBackward;
            if (Math.abs(leftJoyStickSpeedY) >= HIGH_SPEED_THRESHOLD && Math.abs(leftJoyStickSpeedX) <= LOW_SPEED_THRESHOLD) {
                leftJoyStickSpeedX = 0;
            }
            if (Math.abs(leftJoyStickSpeedX) >= HIGH_SPEED_THRESHOLD && Math.abs(leftJoyStickSpeedY) <= LOW_SPEED_THRESHOLD) {
                leftJoyStickSpeedY = 0;
            }
            if (leftJoyStickSpeedX>0 ) {
                strafeBias = strafeBiasRight;
            }
            if (leftJoyStickSpeedY>0) {
                strafeBias = strafeBiasForward;
            }

            follower.setTeleOpMovementVectors(
                    -leftJoyStickSpeedY*speedMultiplier,
                    -leftJoyStickSpeedX*speedMultiplier,
                    -gamepad1.right_stick_x*speedMultiplier +
                            strafeBias*(-leftJoyStickSpeedX*speedMultiplier) +
                            strafeBiasY*(-leftJoyStickSpeedX*speedMultiplier),
                    false);
            follower.update();*/
            telemetry.update();

        }


    }
    public int getFrontSlideTicks() {
        return leftSlideMotor.getCurrentPosition();
    }
}
