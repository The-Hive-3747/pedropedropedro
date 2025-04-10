package pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.examples.FiveSpecimenAuto;
import pedroPathing.subsystem.AutoSpecDrive;
import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;

@Disabled
@TeleOp(name="TeleOpHangTest")
@Config
public class TeleOpHangTest extends LinearOpMode{

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
    private boolean slideWasRetract = false;
    private boolean slideWasExtend = false;
    private boolean slideWasAutoExtend = false;
    private boolean wasPivoting = false;
    private boolean isHolding = false;
    private boolean pivotUpRequested = false;
    private boolean doRelease = false;
    private boolean hangRequested = false;
    private boolean pivotDownRequested = false;
    private boolean hasResetFollowing = false;
    public static double joystickTolerance = 0.1;
    private static double SLIDE_POWER_SCORE_HOLD = 0.3;//0.45;
    public static int SLIDE_LEFT_MIN_LIMIT_BUFFER = 50; //10;
    public static int SLIDE_TOLERANCE = 50;
    public static double SLIDE_POWER = 1.0;//0.9;//0.2;nex
    public static int SLIDE_LEFT_HORIZ_LEGAL_LIMIT_TICKS = 4000;//1008; //1240; //844; //967; //1033;//1444; //1372; //1472;//2027;
    public static int PIVOT_UP_THRESHOLD = 590; //900;
    public static int SLIDE_LEFT_HEIGHT_SCORE_TICKS = 4000;//1936; //3568;
    public static int SLIDE_LEFT_HANG_LIMIT = 4000;//2319;//3962;
    public static int PIVOT_TOLERANCE = 15;
    public static int PIVOT_DOWN_POSITION = 2;
    public static double PIVOT_POWER_UP = 0.7; //0.6; //0.7;
    public static double PIVOT_POWER_HANG = 0.9;//1;
    public static int PIVOT_DOWN_HANG_POSITION = 99;//0;//776;//1400; //1500; //1350; //1245; measured on old motor//1031 actual;//335
    public static int PIVOT_HANG_POSITION = 400;
    public static int PIVOT_HANG2_POSITION = 500;
    public static int PIVOT_HANG_UP_POSITION = 740;
    public static int SLIDE_NO_DOWN_TICKS = 1000;
    public static double PIVOT_POWER_DOWN = 0.3; //0.7; //0.05;
    public boolean hasAutoMoved = false;
    public boolean hasBrokenFollowing = false;
    public boolean hasDriven = false;
    private AutoSpecDrive autoDrive = null;

    private Follower follower;
    @Override
    public void runOpMode() throws InterruptedException{
        specimenArm = new SpecimenArm(hardwareMap, telemetry, false);
        slideArm = new SlideArm(hardwareMap, telemetry, false);
        pivotMotor = hardwareMap.get(DcMotor.class,"pivot_motor");
        leftSlideMotor = hardwareMap.get(DcMotor.class,"left_slide_motor" );
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_slide_motor");
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(FiveSpecimenAuto.startPose);
        autoDrive = new AutoSpecDrive(follower, specimenArm);
        AutoSpecDrive.leftFront = hardwareMap.get(DcMotor.class, FollowerConstants.leftFrontMotorName);
        AutoSpecDrive.rightFront = hardwareMap.get(DcMotor.class,FollowerConstants.rightFrontMotorName);
        AutoSpecDrive.leftBack = hardwareMap.get(DcMotor.class,FollowerConstants.leftRearMotorName);
        AutoSpecDrive.rightBack = hardwareMap.get(DcMotor.class,FollowerConstants.rightRearMotorName);


        waitForStart();

        slideArm.setWristToStow();
        follower.startTeleopDrive();
        while (opModeIsActive()) {
            if(gamepad1.a) {
                specimenArm.goToSpecimenCollect();
            }
            if(gamepad1.y) {
                specimenArm.clawStateOpen();
            }
            if (hasAutoMoved) {
                autoDrive.runSchedule();
                if (autoDrive.isAutoSpecCycleIsDone()) {
                    autoSpecCycle();
                }
            }
            if (gamepad1.b && !hasAutoMoved) {
                autoSpecCycle();
            }
            slideArm.addSlideTelemetry();
            telemetry.update();

            if (gamepad1.b && hasBrokenFollowing) {
                follower.resumePathFollowing();
                hasBrokenFollowing = false;
            }
            if ((Math.abs(gamepad1.left_stick_x) >= joystickTolerance || Math.abs(gamepad1.left_stick_y) >= joystickTolerance || Math.abs(gamepad1.right_stick_x) >= joystickTolerance || Math.abs(gamepad1.right_stick_y) >= joystickTolerance) && !hasDriven) {
                follower.breakFollowing();
                hasDriven = true;
                hasBrokenFollowing = true;
                hasAutoMoved = false;
                follower.startTeleopDrive();

            }

            if (gamepad2.dpad_left) {
                slideWasRetract = true;
                //slideArm.slideDownOneStep();
                doRelease = false;
                if(leftSlideMotor.getCurrentPosition() >= 0){
                    leftSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    leftSlideMotor.setPower(-SLIDE_POWER);
                    rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightSlideMotor.setPower(-SLIDE_POWER);
                }
                /*if(rightSlideMotor.getCurrentPosition() >= LIMIT + SLIDE_TOLERANCE) {
                    rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightSlideMotor.setPower(-SLIDE_POWER);
                }*/

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
                    rightSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    rightSlideMotor.setPower(SLIDE_POWER);
                }

            }
            if (slideWasExtend && !gamepad2.dpad_right) {
                slideWasExtend = false;
                //slideArm.stopSliding();
                //leftSlideMotor.setPower(0);
                //rightSlideMotor.setPower(0);
                if (pivotPosition == SlideArm.PivotState.DOWN && pivotMotor.getCurrentPosition() -PIVOT_TOLERANCE < PIVOT_DOWN_POSITION) {
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
            if (gamepad2.left_bumper) {
                pivotMotor.setPower(0);
                pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            if (gamepad2.dpad_down) {
                slideWasAutoExtend = true;
                hangRequested = false;
                pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition() - 50);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_UP);
                pivotUpRequested = true;
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
                pivotMotor.setTargetPosition(PIVOT_HANG_UP_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_UP);
                pivotUpRequested = true;

            }
            if (gamepad2.a) {
                    /*telemetry.addData("Status", "Pivoting Up");
                    pivotTimer.reset();
                    specimenArm.clawStateClose();
                    slideArm.pivotUp();*/
                slideWasAutoExtend = true;
                //slideArm.scoreAuto();
                hangRequested = false;
                pivotMotor.setTargetPosition(PIVOT_DOWN_HANG_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_HANG);
                pivotUpRequested = true;

            }
            if (gamepad2.b) {
                slideWasAutoExtend = true;
                hangRequested = true;
                pivotPosition = SlideArm.PivotState.UP;
                pivotMotor.setTargetPosition(PIVOT_HANG_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_UP);
                pivotUpRequested = true;
            }

            if (gamepad2.right_bumper) {
                //slideWasAutoExtend = true;
                //hangRequested = true;
                pivotPosition = SlideArm.PivotState.UP;
                pivotMotor.setTargetPosition(PIVOT_HANG2_POSITION);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pivotMotor.setPower(PIVOT_POWER_UP);
                //pivotUpRequested = true;
            }
            /*
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
            }*/
            if (gamepad2.x && !wasPivoting) {
                telemetry.addData("Status", "Pivoting Down");
                //specimenArm.clawStateClose();
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
            if(gamepad1.x){
                slideArm.pivotDownRetract();
            }

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
            follower.update();
            specimenArm.update();
            //slideArm.update();

        }



    }

    public void autoSpecCycle() {
        //follower.breakFollowing();
        //autoDrive.setIntakePose();
        //setIntakePose();
        autoDrive.resetScheduler();
        autoDrive.runSchedule();
        //follower.resumePathFollowing();
        hasAutoMoved = true;
        hasDriven = false;
        hasResetFollowing = false;

    }
    public void setIntakePose() {
        autoDrive.intakePose = follower.getPose().copy();
    }

    public int getFrontSlideTicks() {
        return leftSlideMotor.getCurrentPosition();
    }
}
