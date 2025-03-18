package pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;

@TeleOp(name="TeleOpHangTest")
@Config
public class TeleOpHangTest extends LinearOpMode{
    private SpecimenArm specimenArm = null;
    private SlideArm slideArm = null;
    private DcMotor pivotMotor = null;
    private DcMotor leftSlideMotor = null;
    private DcMotor rightSlideMotor = null;
    private SlideArm.PivotState pivotPosition = SlideArm.PivotState.DOWN;
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
    public static int SLIDE_LEFT_HORIZ_LEGAL_LIMIT_TICKS = 1008; //1240; //844; //967; //1033;//1444; //1372; //1472;//2027;
    public static int PIVOT_UP_THRESHOLD = 590; //900;
    public static int SLIDE_LEFT_HEIGHT_SCORE_TICKS = 1936; //3568;
    public static int SLIDE_LEFT_HANG_LIMIT = 2200;//2319;//3962;
    public static int PIVOT_TOLERANCE = 15;
    public static int PIVOT_DOWN_POSITION = 2;
    public static double PIVOT_POWER_UP = 0.7; //0.6; //0.7;
    public static int PIVOT_UP_POSITION = 740;//776;//1400; //1500; //1350; //1245; measured on old motor//1031 actual;//335
    public static int SLIDE_NO_DOWN_TICKS = 1000;
    public static double PIVOT_POWER_DOWN = 0.3; //0.7; //0.05;
    @Override
    public void runOpMode() throws InterruptedException{
        specimenArm = new SpecimenArm(hardwareMap, telemetry, false);
        slideArm = new SlideArm(hardwareMap, telemetry, false);
        pivotMotor = hardwareMap.get(DcMotor.class,"pivot_motor");
        leftSlideMotor = hardwareMap.get(DcMotor.class,"left_slide_motor" );
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_slide_motor");

        waitForStart();
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
            if (gamepad2.y) {
                    /*telemetry.addData("Status", "Pivoting Up");
                    pivotTimer.reset();
                    specimenArm.clawStateClose();
                    slideArm.pivotUp();*/
                slideWasAutoExtend = true;
                //slideArm.scoreAuto();
                hangRequested = false;
                pivotPosition = SlideArm.PivotState.UP;
                pivotMotor.setTargetPosition(PIVOT_UP_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_UP);
                pivotUpRequested = true;

            }
            if (!gamepad2.y && slideWasAutoExtend) {
                slideWasAutoExtend = false;
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
            if (gamepad2.x && !wasPivoting) {
                telemetry.addData("Status", "Pivoting Down");
                specimenArm.clawStateClose();
                //slideArm.pivotDown();
                if(getFrontSlideTicks()> SLIDE_NO_DOWN_TICKS){
                    return;
                }
                hangRequested = false;
                pivotPosition = SlideArm.PivotState.DOWN;
                pivotMotor.setTargetPosition(PIVOT_DOWN_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_DOWN);
                pivotDownRequested = true;
                wasPivoting = true;
            }
            if (!gamepad2.x && wasPivoting) {
                wasPivoting = false;

            }


        }


    }
    public int getFrontSlideTicks() {
        return leftSlideMotor.getCurrentPosition();
    }
}
