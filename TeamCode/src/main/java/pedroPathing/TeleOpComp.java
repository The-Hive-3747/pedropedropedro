package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystem.AutoSpecDrive;
import pedroPathing.subsystem.IndicatorLight;
import pedroPathing.subsystem.OpModeTransfer;
import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;

import java.util.Stack;

@TeleOp(name="TeleOpComp")
@Config
public class TeleOpComp extends LinearOpMode {
    public static enum TeamColor {TEAM_BLUE, TEAM_RED}
    public static enum SampleColor {SAMPLE_BLUE, SAMPLE_RED, SAMPLE_YELLOW}
    public static TeamColor HIVE_COLOR = TeamColor.TEAM_BLUE;
    public static double speedMultiplier = 0.5;
    public static double strafeBiasLeft = -0.045;
    public static double strafeBiasRight = -0.033;
    public static double strafeBiasForward = 0.0;//-0.05;
    public static double strafeBiasBackward = 0.0;//-0.05;
    public static double HIGH_SPEED_THRESHOLD = 0.8;
    public static double LOW_SPEED_THRESHOLD = 0.1;
    public static double RUMBLE_HANG_TIME_OUT = 90.0;
    public static double RUMBLE_HANG_TIME_OUT_END = 95.0;
    public static double RUMBLE_HANG_TIME2_OUT = 100.0;//105.0;
    public static double RUMBLE_HANG_TIME2_OUT_END = 105.0; //110.0;
    public static double LEVEL_TWO_HANG_RETRACT_MSECONDS = 3000.0;
    public static double LEVEL_TWO_HANG_TIP_MSECONDS = 2500.0;
    public static double SLIDE_HOVER_EXTEND_TIME = 2000.0;
    public static double LEVEL_TWO_HANG_HOOK_MSECONDS = 1000.0; // 2000.0
    public static double LEVEL_TWO_HANG_RELEASE_MSECONDS = 3000.0;
    public static double LEVEL_THREE_HANG_CLIMB_MSECONDS = 12000.0;
    public static double LEVEL_THREE_TIP_TIMER = 6000;
    public static double FLASH_LIGHT2_SECONDS = 0.25;
    public static double FLASH_LIGHT_SECONDS = 0.5;
    public static double SLIDE_LIMIT_CHANGE_TIME = 30000;
    public static Stack<Double> battery_checker = new Stack<>();
    private boolean derailSchedulerReset = false;
    private boolean slideWasRetract = false;
    private boolean slideWasExtend = false;
    private boolean clawWasPushed = false;
    private boolean shoulderWasPushed = false;
    private boolean slideUpHoverL2Started = false;
    private boolean disableSpecimenArmWasPushed = false;
    private boolean releaseSlideArmHangWasPushed = false;
    private boolean configModeActivated = false;
    private boolean derailModeActivated = false;
    private boolean configWasRequested = false;
    private boolean pivotResetWasRequested = false;
    private boolean retractResetWasRequested = false;
    private boolean hiveColorChangeWasRequested = true;
    private boolean teamChangeRequested = false;
    private boolean resetSpeciWasRequested = false;
    private boolean retractResetNeeded = false;
    private boolean pivotResetNeeded = false;
    private boolean speciResetNeeded = false;
    private boolean wasPivoting = false;
    private boolean motorsWereZeroed = false;
    private boolean slideWasAutoExtend = false;
    private boolean aWasPushed = false;
    private boolean isIntaking = false;
    private boolean sitWasPushed = false;
    private boolean rumble1Fired = false;
    private boolean rumble2Fired = false;
    private boolean pivotHangRequested = false;
    private boolean intakeRequested = false;
    private boolean wristChangeRequested = false;
    private boolean isSlowToScore = false;
    private boolean isOutaking = false;
    private boolean clawSensorRan = false;
    private boolean clawSensorWasPressed = false;
    private boolean sensorIntakeRequested = false;
    private boolean showWarningLights = false;
    private boolean noLimitsSlideWasExtend = false;
    private boolean hangModeRequested = false;
    private boolean rawDriveStarted = false;
    private boolean hangButtonWasPressed = false;
    private boolean tensionSchedulerReset = false;
    private boolean levelTwoHangTipped = false;
    private boolean levelTwoHookStarted = false;
    private boolean levelTwoReleaseStarted = false;
    private boolean levelTwoSlideUp = false;
    private boolean levelThreeSlideDown = false;
    private boolean levelThreePivotLimp = false;
    private boolean levelThreeClimb = false;
    private boolean thirdLevelPivotTipped = false;
    private boolean thirdLevelPivotTipRequested = false;
    private boolean hangModeActivated = false;
    private boolean stopTensionRequested = false;
    private boolean pivotLimped = false;
    private boolean slidesChanged = false;
    private boolean backupUsed = false;
    private boolean touchSensorHit = false;
    private double RAW_DRIVE_TIME = 200.0; //250.0;
    private double RAW_DRIVE_BACKWARD_TIME = 800.0;
    private double PIVOT_LIMPED_TIME = 500.0;
    private int TOUCH_SENSOR_TIMER_FLASH_TIME = 1000;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private double DRIVE_POWER = 0.3;
    private SpecimenArm specimenArm = null;
    private ElapsedTime driveTime = new ElapsedTime();
    private ElapsedTime matchTimer = new ElapsedTime();
    private ElapsedTime shoulderTimer = new ElapsedTime();
    private SlideArm slideArm = null;
    private ElapsedTime pivotTimer = new ElapsedTime();
    private ElapsedTime wristTimer = new ElapsedTime();
    private ElapsedTime resetPivotTimer = new ElapsedTime();
    private ElapsedTime resetSlideTimer = new ElapsedTime();
    private ElapsedTime resetSpeciTimer = new ElapsedTime();
    private ElapsedTime criticalLoopTimer = new ElapsedTime();
    private ElapsedTime slowDownTimer = new ElapsedTime();
    private ElapsedTime flashLightTimer = new ElapsedTime();
    private ElapsedTime touchLightTimer = new ElapsedTime();
    //private static Pose2d RESET_POSE = new Pose2d(0.0, 0.0, 0.0);
    private static double pivotTimerThreshold = 1000.0;
    private static double wristTimerThreshold = 750.0;
    private static double shoulderTimerThreshold = 500.0;
    private static double resetPivotTimerThreshold = 1000.0;
    private static double resetSlideTimerThreshold = 1000.0;
    private static double resetSpeciTimerThreshold = 1000.0;
    private static double SPEED_TIME_SLOW_FOR_SCORE_THRESHOLD = SpecimenArm.SPECIMENARM_SCORE_TIME + 10.0;
    public static float LEFT_TRIGGER_THRESHOLD = 0.7f;
    public static float RIGHT_TRIGGER_THRESHOLD = 0.7f;
    private static double STICK_X_LEFT = -0.6;
    private static double STICK_X_RIGHT = 0.6;
    private static double NORMAL_SPEED = 0.7; //0.8;
    private static double LUDICROUS_SPEED = 1.0;
    private static double SLOW_SPEED = 0.5;
    private static double SLOW_SCORE_SPEED = 0.2;
    private boolean hasAutoTensioned = false;
    private double maxLoopTime = 0.0;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);
    private boolean resetMotors = false;
    private boolean hasTensioned = false;
    private boolean hasDownTensioned = false;
    public static int hangState = -1;
    private boolean firstPhaseStarted = false;
    private boolean pivotStarted = false;
    private ElapsedTime levelTwoHangTimer = new ElapsedTime();
    private boolean levelTwoHangStarted = false;
    private boolean nextStateRequested = false;
    private AutoSpecDrive autoDrive = null;
    public boolean hasAutoMoved = false;
    public boolean hasBrokenFollowing = false;
    public boolean hasDriven = false;
    public boolean hasResetFollowing = false;
    public static double joystickTolerance = 0.1;
    @Override
    public void runOpMode() throws InterruptedException {
        hangState = -1;
        derailModeActivated = false;
        hangModeActivated = false;
        hasAutoTensioned = false;
        HIVE_COLOR = OpModeTransfer.autoColor;
        IndicatorLight leftLight = new IndicatorLight(hardwareMap, telemetry, "left_light");
        IndicatorLight rightLight = new IndicatorLight(hardwareMap, telemetry, "right_light");

        leftLight.setColor(IndicatorLight.COLOR_RED);
        rightLight.setColor(IndicatorLight.COLOR_RED);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(OpModeTransfer.autoPose);
        leftFront = hardwareMap.get(DcMotor.class, FollowerConstants.leftFrontMotorName);
        rightFront = hardwareMap.get(DcMotor.class,FollowerConstants.rightFrontMotorName);
        leftBack = hardwareMap.get(DcMotor.class,FollowerConstants.leftRearMotorName);
        rightBack = hardwareMap.get(DcMotor.class,FollowerConstants.rightRearMotorName);
        autoDrive = new AutoSpecDrive(follower, specimenArm);
        AutoSpecDrive.leftFront = hardwareMap.get(DcMotor.class, FollowerConstants.leftFrontMotorName);
        AutoSpecDrive.rightFront = hardwareMap.get(DcMotor.class,FollowerConstants.rightFrontMotorName);
        AutoSpecDrive.leftBack = hardwareMap.get(DcMotor.class,FollowerConstants.leftRearMotorName);
        AutoSpecDrive.rightBack = hardwareMap.get(DcMotor.class,FollowerConstants.rightRearMotorName);

        //PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

        specimenArm = new SpecimenArm(hardwareMap, telemetry, false);
        slideArm = new SlideArm(hardwareMap, telemetry, false);
        //leftLight.setColor(IndicatorLight.COLOR_GREEN);
        //rightLight.setColor(IndicatorLight.COLOR_GREEN);
        if (HIVE_COLOR == TeleOpComp.TeamColor.TEAM_BLUE) {
            leftLight.setColor(IndicatorLight.COLOR_BLUE);
            rightLight.setColor(IndicatorLight.COLOR_BEECON);
            hiveColorChangeWasRequested = true;
        } else {
            leftLight.setColor(IndicatorLight.COLOR_RED);
            rightLight.setColor(IndicatorLight.COLOR_BEECON);
        }
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.back && !resetMotors) {
                resetMotors = true;
                rightLight.setColor(IndicatorLight.COLOR_VIOLET);
                slideArm.resetPivotEncoders();
                slideArm.resetSlideEncoders();
                specimenArm.resetSpecimenEncoders();
            }
            telemetry.addData("Press Gamepad1 back to reset all motor 0s", "");
            telemetry.update();
        }

        waitForStart();
        matchTimer.reset();
        slideArm.setWristToReady();
        // slideArm.derailShiftDown(); // TODO: UNCOMMENT THIS FOR WORLDS
        follower.startTeleopDrive();


        criticalLoopTimer.reset();
        while (opModeIsActive()) {
            if (configModeActivated){
                runConfigMode();
            }
            else if (derailModeActivated) {
                if (slideArm.hasDerailed() && !slideArm.isTensionDone()) {
                    slideArm.tensionSlides();
                    if (!hasAutoTensioned) {
                        hasAutoTensioned = true;
                        slideArm.slideDownDerail();
                        slideArm.derailShiftSecond();
                        specimenArm.goToSpecimenCollect();
                    }
                }
                if (!hangModeActivated) {
                    runDerailMode();
                }
                else {
                    runHangMode();
                }
            }
            else {
                    if (hasAutoMoved) {
                        autoDrive.runSchedule();
                        if (autoDrive.isAutoSpecCycleIsDone()) {
                            autoSpecCycle();
                        }
                    }
                    if (gamepad1.dpad_down && !hasAutoMoved) {
                        autoSpecCycle();
                    }
                    if (gamepad1.dpad_down && hasBrokenFollowing) {
                        //follower.resumePathFollowing();
                        hasBrokenFollowing = false;
                    }
                       if ((Math.abs(gamepad1.left_stick_x) >= joystickTolerance || Math.abs(gamepad1.left_stick_y) >= joystickTolerance || Math.abs(gamepad1.right_stick_x) >= joystickTolerance || Math.abs(gamepad1.right_stick_y) >= joystickTolerance) && !hasDriven) {
                        follower.breakFollowing();
                        hasDriven = true;
                        hasBrokenFollowing = true;
                        hasAutoMoved = false;
                        follower.startTeleopDrive();
                    }
                    if (gamepad2.dpad_down && hangModeRequested && !derailModeActivated && !hangButtonWasPressed) {
                        if (gamepad2.back) {
                            hangModeActivated = true;
                        }
                        else {
                            derailModeActivated = true;
                        }
                    }
                    if (gamepad2.dpad_down && !hangButtonWasPressed) {
                        if (!hangModeRequested) {
                            hangModeRequested = true;
                        }
                        hangButtonWasPressed = true;
                    }
                    else if (!gamepad2.dpad_down) {
                        hangButtonWasPressed = false;
                    }

                    if (hangModeRequested && nonHangButtonWasPressed()) {
                        hangModeRequested = false;
                    }
                    if (gamepad2.left_stick_button && !disableSpecimenArmWasPushed) {
                        specimenArm.disableSpecimenArm();
                        disableSpecimenArmWasPushed = true;
                    } else if (!gamepad2.left_stick_button && disableSpecimenArmWasPushed){
                        disableSpecimenArmWasPushed = false;
                    }

                    if (gamepad2.right_stick_button && gamepad2.dpad_right) {
                        noLimitsSlideWasExtend = true;
                        slideArm.slideUpOneStepWithoutLimits();
                    }

                    if (!gamepad2.right_stick_button && noLimitsSlideWasExtend) {
                        noLimitsSlideWasExtend = false;
                        slideArm.stopSliding();
                    }
                /*if (gamepad1.x && shoulderTimer.milliseconds() > shoulderTimerThreshold) {
                    telemetry.addData("Status", "Shoulder Moved");
                    shoulderTimer.reset();
                    specimenArm.goToNextSpecimenState();
                }*/
                    if (matchTimer.seconds() >= RUMBLE_HANG_TIME_OUT && !rumble1Fired) {
                        rumble1Fired = true;
                        gamepad1.rumble(300);
                        gamepad2.rumble(300);
                    }else if (matchTimer.seconds() >= RUMBLE_HANG_TIME2_OUT && !rumble2Fired){
                        rumble2Fired = true;
                        gamepad1.rumble(300);
                        gamepad2.rumble(300);
                    }
                    if (matchTimer.seconds() >= RUMBLE_HANG_TIME_OUT && matchTimer.seconds() <= RUMBLE_HANG_TIME_OUT_END) {
                        if (flashLightTimer.seconds() >= FLASH_LIGHT_SECONDS) {
                            showWarningLights = !showWarningLights;
                            flashLightTimer.reset();
                        }
                    }
                    if (matchTimer.seconds() >= RUMBLE_HANG_TIME_OUT_END && matchTimer.seconds() <= RUMBLE_HANG_TIME2_OUT) {
                        showWarningLights = false;
                    }
                    if (matchTimer.seconds() >= RUMBLE_HANG_TIME2_OUT && matchTimer.seconds() <= RUMBLE_HANG_TIME2_OUT_END) {
                        if (flashLightTimer.seconds() >= FLASH_LIGHT2_SECONDS) {
                            showWarningLights = !showWarningLights;
                            flashLightTimer.reset();
                        }
                    }
                    if (matchTimer.seconds() >= RUMBLE_HANG_TIME2_OUT_END) {
                        showWarningLights = false;
                    }
                    if (gamepad1.x && !shoulderWasPushed) {
                        if (specimenArm.getShoulderState() == SpecimenArm.ShoulderState.ENTER){
                            isSlowToScore = true;
                            slowDownTimer.reset();
                        }
                        specimenArm.goToNextSpecimenState();
                        shoulderWasPushed = true;
                    } else if (!gamepad1.x && shoulderWasPushed) {
                        shoulderWasPushed = false;
                    }
                    /*
                    if (gamepad2.b && !pivotHangRequested) {
                        //hangModeRequested = true;
                        slideArm.pivotForTallHang();
                        specimenArm.clawStateClose();
                        specimenArm.goToSpecimenCollect();
                        slideArm.setWristToStow();
                        pivotHangRequested = true;
                        derailModeActivated = true;
                    }
                    if (!gamepad2.b && pivotHangRequested) {
                        pivotHangRequested = false;
                    }*/
                    if (gamepad2.a && !aWasPushed && derailModeActivated) {
                        aWasPushed = true;
                        slideArm.pivotDownForTallHang();
                        slideArm.hangHoldForTallHang();
                    }
                    if (matchTimer.milliseconds() >= SLIDE_LIMIT_CHANGE_TIME && !slidesChanged) {
                        slidesChanged = true;
                        SlideArm.SLIDE_LEFT_HEIGHT_SCORE_TICKS = 2100;
                        SlideArm.SLIDE_RIGHT_HEIGHT_SCORE_TICKS = 2100;
                    }
                    if (!gamepad2.a && aWasPushed){
                        aWasPushed = false;
                        //slideArm.stopHang();
                    }
                    /*if (gamepad2.dpad_up && !sitWasPushed){
                        sitWasPushed = true;
                        slideArm.sit();
                    }
                    if (!gamepad2.dpad_up && sitWasPushed){
                        sitWasPushed = false;
                    }*/
                    if (gamepad1.y && !clawWasPushed) {
                        specimenArm.nextClawState();
                        backupUsed = true;
                        touchLightTimer.reset();
                        clawWasPushed = true;
                    } else if (!gamepad1.y && clawWasPushed) {
                        clawWasPushed = false;
                    }
                    if (gamepad1.back && !releaseSlideArmHangWasPushed) {
                        slideArm.hangRelease();
                        releaseSlideArmHangWasPushed = true;
                    } else if (!gamepad1.back && releaseSlideArmHangWasPushed) {
                        releaseSlideArmHangWasPushed = false;
                    }
                    if (gamepad2.dpad_left) {
                        slideWasRetract = true;
                        slideArm.slideDownOneStep();
                    }
                    if (slideWasRetract && !gamepad2.dpad_left) {
                        slideWasRetract = false;
                        slideArm.stopSliding();
                    }
                    if (gamepad2.dpad_right && !gamepad2.right_stick_button) {
                        slideWasExtend = true;
                        slideArm.slideUpOneStep();
                    }
                    if (slideWasExtend && !gamepad2.dpad_right) {
                        slideWasExtend = false;
                        slideArm.stopSliding();
                    }
                    if (!clawSensorRan && !clawSensorWasPressed && specimenArm.clawSensor.isPressed()) {
                        //specimenArm.clawSensorGrab();
                        specimenArm.clawStateClose();
                        touchSensorHit = true;
                        touchLightTimer.reset();
                        sleep(SpecimenArm.CLAW_GRAB_DELAY);
                        specimenArm.goToSpecimenEnter();
                        driveTime.reset();
                        rawDriveForward();
                        clawSensorRan = true;
                    }
                    if (clawSensorRan && !clawSensorWasPressed && driveTime.milliseconds() > RAW_DRIVE_TIME){
                        clawSensorWasPressed = true;
                        stopDriveForward();
                    }
                    if (clawSensorRan && clawSensorWasPressed && !specimenArm.clawSensor.isPressed()){
                        clawSensorWasPressed = false;
                        clawSensorRan = false;
                    }
                    if (gamepad2.y) {
                    /*telemetry.addData("Status", "Pivoting Up");
                    pivotTimer.reset();
                    specimenArm.clawStateClose();
                    slideArm.pivotUp();*/
                        slideWasAutoExtend = true;
                        slideArm.scoreAuto();

                    }
                    if (!gamepad2.y && slideWasAutoExtend){
                        slideWasAutoExtend = false;
                        slideArm.stopSliding();
                    }
                    if (gamepad2.x && !wasPivoting) {
                        telemetry.addData("Status", "Pivoting Down");
                        pivotTimer.reset();
                        specimenArm.clawStateClose();
                        slideArm.pivotDown();
                        wasPivoting = true;
                    }
                    if (!gamepad2.x && wasPivoting){
                        wasPivoting = false;
                    }
                    if (gamepad1.left_bumper && gamepad1.right_bumper) {
                        slideArm.makeLimp();
                        specimenArm.makeLimp();
                    }
                    if (gamepad2.right_bumper && !intakeRequested) {
                        isIntaking = true;
                        if (!slideArm.isWristGather() && !slideArm.isPivotUp()){
                            slideArm.setWristToGather();
                        }
                        slideArm.activateIntakeWithoutSensor();
                        intakeRequested = true;
                        //leftLight.setColor(leftLight.COLOR_SAGE);
                        //rightLight.setColor(rightLight.COLOR_SAGE);
                    }
                    if (gamepad2.left_bumper && !isIntaking) {
                        isIntaking = slideArm.activateIntakeWithSensor();
                        sensorIntakeRequested = true;
                        //leftLight.setColor(IndicatorLight.COLOR_BEECON);
                        //slideArm.isKeeperBlock();
                        //slideArm.activateIntakeWithSensor();
                    }
                    if (!gamepad2.left_bumper && sensorIntakeRequested) {
                        sensorIntakeRequested = false;
                        isIntaking = false;
                        slideArm.stopIntake();
                    }
                    if (!gamepad2.right_bumper && intakeRequested) {
                        isIntaking = false;
                        slideArm.stopIntake();
                        intakeRequested = false;
                    }
                    if (gamepad2.right_trigger > RIGHT_TRIGGER_THRESHOLD && !isOutaking) {
                        isOutaking = true;
                        slideArm.reverseIntakeWithoutSensor();
                    }
                    if (gamepad2.right_trigger < RIGHT_TRIGGER_THRESHOLD && isOutaking) {
                        isOutaking = false;
                        slideArm.stopIntake();
                    }
                /*
                if (gamepad2.start) {
                    slideArm.slowMoveMotors(true);
                }*/
                    if (gamepad2.left_trigger > LEFT_TRIGGER_THRESHOLD && !wristChangeRequested) {//wristTimer.milliseconds() > wristTimerThreshold) {
                        wristTimer.reset();
                        slideArm.nextWristPosition();
                        wristChangeRequested = true;
                    }
                    if (gamepad2.left_trigger <= LEFT_TRIGGER_THRESHOLD && wristChangeRequested) {
                        wristChangeRequested = false;
                    }
                    if (gamepad1.a) {
                        //TODO Use Normal hang hold
                        slideArm.firstHangHold();
                    }
                    /*if (gamepad1.dpad_right){
                        //TODO Use Normal Pivot
                        slideArm.firstHangRequested = true;
                        slideArm.firstPivot();
                    }*/
                    if (gamepad1.b) {
                        follower.setPose(startPose);
                    }
                }

            if (gamepad1.dpad_up && !configWasRequested) {
                configModeActivated = !configModeActivated;
                configWasRequested = true;
            } else if (!gamepad1.dpad_up && configWasRequested) {
                configWasRequested = false;
            }
            speedMultiplier = NORMAL_SPEED;
            if (isSlowToScore && slowDownTimer.milliseconds() < SPEED_TIME_SLOW_FOR_SCORE_THRESHOLD){
                speedMultiplier = SLOW_SCORE_SPEED;
            }else if (gamepad1.left_bumper){
                speedMultiplier = SLOW_SPEED;
            }else if (isSlowToScore && slowDownTimer.milliseconds() > SPEED_TIME_SLOW_FOR_SCORE_THRESHOLD){
                isSlowToScore = false;
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

            if (configModeActivated){
                if (HIVE_COLOR == TeamColor.TEAM_BLUE){
                    leftLight.setColor(IndicatorLight.COLOR_BLUE);
                    rightLight.setColor(IndicatorLight.COLOR_ORANGE);
                    hiveColorChangeWasRequested = true;
                }else{
                    leftLight.setColor(IndicatorLight.COLOR_RED);
                    rightLight.setColor(IndicatorLight.COLOR_ORANGE);
                }
                if (motorsWereZeroed){
                    rightLight.setColor(IndicatorLight.COLOR_VIOLET);
                }
            }
            else if (showWarningLights) {
                leftLight.setColor(IndicatorLight.COLOR_RED);
                rightLight.setColor(IndicatorLight.COLOR_RED);
            }
            else if (hangState>3 && hangState < 10){
                leftLight.setColor(IndicatorLight.COLOR_ORANGE);
                rightLight.setColor(IndicatorLight.COLOR_VIOLET);
            }
            else if (hangModeActivated) {
                leftLight.setColor(IndicatorLight.COLOR_VIOLET);
                rightLight.setColor(IndicatorLight.COLOR_VIOLET);
            }
            else if (derailModeActivated){
                leftLight.setColor(IndicatorLight.COLOR_SAGE);
                rightLight.setColor(IndicatorLight.COLOR_SAGE);
            }
            else if (hangModeRequested) {
                leftLight.setColor(IndicatorLight.COLOR_VIOLET);
            }
            else if (noLimitsSlideWasExtend) {
                leftLight.setColor(IndicatorLight.COLOR_BLUE);
                rightLight.setColor(IndicatorLight.COLOR_BLUE);
            }
            else if (touchSensorHit) {
                if (TOUCH_SENSOR_TIMER_FLASH_TIME < touchLightTimer.milliseconds()) {
                    touchSensorHit = false;
                }
                rightLight.setColor(IndicatorLight.COLOR_GREEN);
                leftLight.setColor(IndicatorLight.COLOR_BEECON);
            }
            else if (backupUsed) {
                if (TOUCH_SENSOR_TIMER_FLASH_TIME < touchLightTimer.milliseconds()) {
                    backupUsed = false;
                }
                rightLight.setColor(IndicatorLight.COLOR_RED);
                leftLight.setColor(IndicatorLight.COLOR_BEECON);
            }
            else if (isIntaking && slideArm.isWristGather()){
                if (HIVE_COLOR == TeamColor.TEAM_BLUE) {
                    leftLight.setColor(IndicatorLight.COLOR_BLUE);
                    rightLight.setColor(IndicatorLight.COLOR_SAGE);
                    hiveColorChangeWasRequested = true;
                }
                else if (HIVE_COLOR ==TeamColor.TEAM_RED) {
                    leftLight.setColor(IndicatorLight.COLOR_RED);
                    rightLight.setColor(IndicatorLight.COLOR_SAGE);
                    hiveColorChangeWasRequested = true;
                }
            }
            else{
                leftLight.setColor(IndicatorLight.COLOR_BEECON);
                rightLight.setColor(IndicatorLight.COLOR_BEECON);
            }


            slideArm.update();
            specimenArm.update();
            if (maxLoopTime < criticalLoopTimer.milliseconds()){
                maxLoopTime = criticalLoopTimer.milliseconds();
            }
            //NEED to add if statement for hiveColorChangeRequested below:
            if(hiveColorChangeWasRequested) {
                telemetry.addData("Team Color", HIVE_COLOR);
            }
            telemetry.addData("Config Mode Activated", configModeActivated);
            telemetry.addData("Critical Loop MS", criticalLoopTimer.milliseconds());
            criticalLoopTimer.reset();
            telemetry.addData("Max Critical Loop Time", maxLoopTime);
            //telemetry.addData("Intake Distance", slideArm.getIntakeDistaceCM());
            //telemetry.addData("Intake Color:", slideArm.detectColor());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Intake Status", slideArm.getIntakeStatus());
            telemetry.addData("Hang Requested", slideArm.hangRequested);
            telemetry.addData("Wrist Status", slideArm.getWristPosition());
            slideArm.addSlideTelemetry();
            telemetry.addData("Right Shoulder Ticks", specimenArm.rightShoulder.getCurrentPosition());
            telemetry.addData("Claw Touch Sensor", specimenArm.clawSensor.isPressed());

            telemetry.update();
            hiveColorChangeWasRequested = false;

            // TODO: IF FTC DASHBOARD NEEDED: UNCOMMENT TELEMETRYPACKET & FTCDASHBOARD

            //TelemetryPacket packet = new TelemetryPacket();

            //packet.fieldOverlay().setStroke("#3F51B5");
            //Drawing.drawRobot(packet.fieldOverlay(), drive.pose);

            //FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
    public boolean nonHangButtonWasPressed() {
        return (gamepad2.x ||
                gamepad2.y ||
                gamepad2.a ||
                gamepad2.b ||
                gamepad2.dpad_left ||
                gamepad2.dpad_up ||
                gamepad2.dpad_right ||
                gamepad2.right_bumper ||
                gamepad2.left_bumper ||
                gamepad2.left_trigger >= LEFT_TRIGGER_THRESHOLD ||
                gamepad2.right_trigger >= RIGHT_TRIGGER_THRESHOLD );
    }
    public void rawDriveForward(){
        follower.breakFollowing();
        leftBack.setPower(DRIVE_POWER);
        rightBack.setPower(DRIVE_POWER);
        leftFront.setPower(DRIVE_POWER);
        rightFront.setPower(DRIVE_POWER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void rawDriveBackward(){
        follower.breakFollowing();
        leftBack.setPower(-DRIVE_POWER);
        rightBack.setPower(-DRIVE_POWER);
        leftFront.setPower(-DRIVE_POWER);
        rightFront.setPower(-DRIVE_POWER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void stopDriveForward() {
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        follower.startTeleopDrive();
    }
    public void restartTeleopDrive() {
        follower.startTeleopDrive();
    }

    public void runDerailMode() {

        if (!slideArm.hasDerailed() && !slideWasExtend && !slideWasRetract) {
            if (!derailSchedulerReset) {
                showWarningLights = false;
                slideArm.derailResetScheduler();
                derailSchedulerReset = true;
            }
            slideArm.runSchedule();
        }

        if (slideArm.hasDerailed()) {
            hangModeActivated = true;
        }

        if (gamepad2.dpad_left) {
            slideWasRetract = true;
            slideArm.slideDownOneStep();
        }
        if (slideWasRetract && !gamepad2.dpad_left) {
            slideWasRetract = false;
            slideArm.stopSliding();
        }
        if (gamepad2.dpad_right) {
            slideWasExtend = true;
            slideArm.slideUpOneStep();
        }
        if (slideWasExtend && !gamepad2.dpad_right) {
            slideWasExtend = false;
            slideArm.stopSliding();
        }
    }

    public void runHangMode() {
        //Back up 2 inches
        //Pivot at an angle to hang
        //Extend slides
        //Pivot Down to hit the bar
        //Pull slides in


        switch(hangState){
            case -1:
                if (gamepad2.dpad_left) {
                    slideWasRetract = true;
                    slideArm.retractWithoutLimits();
                }
                if (slideWasRetract && !gamepad2.dpad_left) {
                    slideWasRetract = false;
                    slideArm.resetSlideEncoders();
                    slideArm.stopSliding();
                }
                if (gamepad2.dpad_right) {
                    slideWasExtend = true;
                    slideArm.slideUpOneStep();
                }
                if (slideWasExtend && !gamepad2.dpad_right) {
                    slideWasExtend = false;
                    slideArm.stopSliding();
                }
                if (!stopTensionRequested && gamepad2.a) {
                    stopTensionRequested = true;
                }
                if (stopTensionRequested && gamepad2.b) {
                    stopTensionRequested = false;
                }

                if(gamepad2.back && !nextStateRequested) {
                    nextStateRequested = true;
                }
                if(!gamepad2.back && nextStateRequested) {
                    nextStateRequested = false;
                    pivotStarted = false;
                    hangState++;
                }
                break;

            case 0: // pivot up, first level
                if(!pivotStarted) {
                    slideArm.setWristToStow();
                    specimenArm.goToSpecimenCollect();
                    specimenArm.clawStateOpen();
                    slideArm.zeroMotors();
                    slideArm.pivotHang();
                    pivotStarted = true;
                }
                if (!slideArm.pivotIsBusy()) {
                    slideUpHoverL2Started = false;
                    hangState++;
                }
                break;
            case 1: // extend onto first bar
                if(!slideUpHoverL2Started) {
                    levelTwoHangTimer.reset();
                    slideUpHoverL2Started = true;
                    slideArm.slideUpHoverLevel2();
                }
                /*if (gamepad2.dpad_right && !slideWasExtend) {
                    slideWasExtend = true;
                    slideArm.slideUpOneStepWithoutLimitsHang();
                }
                if (slideWasExtend && !gamepad2.dpad_right) {
                    slideWasExtend = false;
                    slideArm.stopSliding();
                }
                if(gamepad2.back && !nextStateRequested) {
                    nextStateRequested = true;
                }
                if(!gamepad2.back && nextStateRequested) {
                    nextStateRequested = false;
                    hangState++;
                }*/
                if (!slideArm.slidesAreBusy() || levelTwoHangTimer.milliseconds() > SLIDE_HOVER_EXTEND_TIME) {
                    nextStateRequested = false;
                    hangState++;
                }
                break;
            case 2: // fall onto first bar
                if (!pivotLimped) {
                    pivotLimped = true;
                    levelTwoHangTimer.reset();
                    slideArm.makePivotLimp();
                    specimenArm.disableSpecimenArm();
                }
                if (levelTwoHangTimer.milliseconds() > RAW_DRIVE_BACKWARD_TIME) {
                    rawDriveStarted = false;
                    hangState++;
                }
                break;
            case 3:

                if (!rawDriveStarted) {
                    rawDriveStarted = true;
                    driveTime.reset();
                    rawDriveBackward();
                    slideArm.slideUpRawDrive();
                }

                if (driveTime.milliseconds() >= RAW_DRIVE_BACKWARD_TIME) {
                    slideArm.releaseSlides();
                    stopDriveForward();
                    hangState++;
                }

                break;
            case 4: // retract into first bar
                /*if (gamepad2.dpad_left && !slideWasRetract) {
                    slideWasRetract = true;
                    slideArm.slideDownOneStepWithoutLimitsHang();
                }
                if (slideWasRetract && !gamepad2.dpad_left) {
                    slideWasRetract = false;
                    slideArm.stopSliding();
                }
                if (gamepad2.dpad_right && !slideWasExtend) {
                    slideWasExtend = true;
                    slideArm.slideUpOneStepWithoutLimitsHang();
                }
                if (slideWasExtend && !gamepad2.dpad_right) {
                    slideWasExtend = false;
                    slideArm.stopSliding();
                }

                if(gamepad2.back && !nextStateRequested) {
                    nextStateRequested = true;
                }
                if(!gamepad2.back && nextStateRequested) {
                    nextStateRequested = false;
                    levelTwoHangStarted = false;
                    levelTwoHangTipped = false;
                    hangState++;
                }*/
                levelTwoHangStarted = false;
                levelTwoHangTipped = false;
                hangState++;
                break;
            case 5: // LEVEL TWO HANG
                if(!levelTwoHangStarted) {
                    levelTwoHangTimer.reset();
                    levelTwoHangStarted = true;
                    slideArm.pivotLevelTwoHang();
                    slideArm.slideDownOneStepWithoutLimitsHang();
                }
                if(!levelTwoHangTipped && levelTwoHangTimer.milliseconds() >= LEVEL_TWO_HANG_TIP_MSECONDS){
                    levelTwoHangTipped = true;
                    slideArm.pivotLevelTwoTip();
                }
                if (levelTwoHangTimer.milliseconds() >= LEVEL_TWO_HANG_RETRACT_MSECONDS) {
                    slideArm.stopSliding();
                    hangState++;
                }
                break;
            case 6: // pivot onto hook
                /*if(!levelTwoHookStarted){
                    levelTwoHookStarted = true;
                    slideArm.pivotLevelTwoHook();
                    levelTwoHangTimer.reset();
                }
                if(levelTwoHangTimer.milliseconds() >= LEVEL_TWO_HANG_HOOK_MSECONDS){
                    hangState++;
                }*/
                levelTwoReleaseStarted = false;
                hangState++;
                break;
            case 7: // release onto hook
                /*if(!levelTwoReleaseStarted){
                    levelTwoReleaseStarted = true;
                    //slideArm.releaseSlides();
                    //slideArm.makePivotLimp();
                    levelTwoHangTimer.reset();
                }
                if(levelTwoHangTimer.milliseconds() >= LEVEL_TWO_HANG_RELEASE_MSECONDS){
                    //hangState++;
                }*/
                if (!levelTwoReleaseStarted) {
                    slideArm.pivotBrakes();
                    levelTwoReleaseStarted = true;
                    }
                break;

                //TODO NEVER REACHES CASE 8 FOR LOW LEVEL HANG
            /*case 8:
                if(!levelTwoSlideUp){
                    levelTwoSlideUp = true;
                    slideArm.zeroMotors();
                    slideArm.holdPivot();
                    slideArm.slideUpForHang();
                    levelTwoHangTimer.reset();
                }
                if(levelTwoHangTimer.milliseconds() >= LEVEL_TWO_HANG_RELEASE_MSECONDS){
                    hangState++;
                }
                break;
            case 9:
                if(!levelThreeSlideDown){
                    levelThreeSlideDown = true;
                    slideArm.slideDownToBar();
                    levelTwoHangTimer.reset();
                }
                if(gamepad2.back && !nextStateRequested) {
                    nextStateRequested = true;
                }
                if(!gamepad2.back && nextStateRequested) {
                    nextStateRequested = false;
                    hangState++;
                }
                if (gamepad2.dpad_left && !slideWasRetract) {
                    slideWasRetract = true;
                    slideArm.slideDownOneStepWithoutLimitsHang();
                }
                if (slideWasRetract && !gamepad2.dpad_left) {
                    slideWasRetract = false;
                    slideArm.stopSliding();
                }
                if (gamepad2.dpad_right && !slideWasExtend) {
                    slideWasExtend = true;
                    slideArm.slideUpOneStepWithoutLimitsHang();
                }
                if (slideWasExtend && !gamepad2.dpad_right) {
                    slideWasExtend = false;
                    slideArm.stopSliding();
                }
                break;
            case 10:
                if(!levelThreePivotLimp){
                    levelThreePivotLimp = true;
                    slideArm.makeHangPivotLimp();
                    levelTwoHangTimer.reset();
                }
                if (gamepad2.dpad_left && !slideWasRetract) {
                    slideWasRetract = true;
                    slideArm.slideDownOneStepWithoutLimitsHang();
                }
                if (slideWasRetract && !gamepad2.dpad_left) {
                    slideWasRetract = false;
                    slideArm.stopSliding();
                }
                if (gamepad2.dpad_right && !slideWasExtend) {
                    slideWasExtend = true;
                    slideArm.slideUpOneStepWithoutLimitsHang();
                }
                if (slideWasExtend && !gamepad2.dpad_right) {
                    slideWasExtend = false;
                    slideArm.stopSliding();
                }
                if(gamepad2.back && !nextStateRequested) {
                    nextStateRequested = true;
                }
                if(!gamepad2.back && nextStateRequested) {
                    nextStateRequested = false;
                    hangState++;
                }
                break;
            case 11:
                if(!levelThreeClimb){
                    levelThreeClimb = true;
                    slideArm.slideDownFullPower();
                    levelTwoHangTimer.reset();
                }
                if(levelTwoHangTimer.milliseconds() >= LEVEL_THREE_HANG_CLIMB_MSECONDS){
                    slideArm.stopSliding();
                    hangState++;
                }
                if(!thirdLevelPivotTipRequested && gamepad2.x){
                    thirdLevelPivotTipRequested = true;
                    slideArm.pivotDownThirdLevelHang();
                }
                if(gamepad2.y) {
                    slideArm.slideUpSmallPower();
                }
                if (gamepad2.a) {
                    slideArm.slideBrakes();
                }
                if (gamepad2.b) {
                    slideArm.slideDownFullPower();
                }
                break;
            case 12:
                break;*/


            default:
                telemetry.addData("HANG STATE WENT WRONG","");
        }


        telemetry.addData("Hangstate: ",hangState);

        //slideArm.runSchedule();
    }


    public void runConfigMode(){
        telemetry.addData("Gamepad2.x", "Pivot Down No Limits");
        telemetry.addData("Gamepad2 dpad left", "Retract No Limits");
        telemetry.addData("Gamepad2 b", "Tension");
        telemetry.addData("Gamepad2 b & dpad down / dpad up", "Tension right slide / left slide");
        telemetry.addData("Gamepad2 y", "loosen tensioner");
        telemetry.addData("Gamepad2 y & dpad down / dpad up", "Loosen right slide / left slide");
        telemetry.addData("Gamepad2 a", "Derail Shift");
        telemetry.addData("Gamepad1 dpad right", "Change Team");
        telemetry.addData("Gamepad1 guide (Logo Button)", "Start Teleop Drive");
        telemetry.addData("Gamepad1 x", "Reset Speciarm");
        telemetry.addData("Gamepad1 start", "Zero Everything");
        if (gamepad2.x) {
            pivotResetWasRequested = true;
            slideArm.pivotDownWithoutLimits();
            resetPivotTimer.reset();
        }
        if (!gamepad2.x && pivotResetWasRequested) {
            pivotResetWasRequested = false;
            pivotResetNeeded = true;
            slideArm.resetPivotEncoders();
        }

        if (gamepad2.dpad_left){
            retractResetWasRequested = true;
            slideArm.retractWithoutLimits();
            resetSlideTimer.reset();
        }
        if (!gamepad2.dpad_left && retractResetWasRequested) {
            retractResetWasRequested = false;
            retractResetNeeded = true;
            slideArm.resetSlideEncoders();
        }
        if (gamepad2.a) {
            slideArm.derailShiftFirst();
        }
        if(gamepad2.dpad_down && gamepad2.b) {
            slideArm.stopTensioning();
            slideArm.tensionRightSlide = true;
            slideArm.tensionSlidesIndividual();
            hasTensioned = true;
        }
        if(gamepad2.dpad_up && gamepad2.b) {
            slideArm.stopTensioning();
            slideArm.tensionRightSlide = false;
            slideArm.tensionSlidesIndividual();
            hasTensioned = true;
        }
        if (gamepad2.b && !gamepad2.dpad_down && !gamepad2.dpad_up) {
            slideArm.tensionSlidesManual();
            hasTensioned = true;
        }
        if (!gamepad2.b && hasTensioned) {
            slideArm.stopTensioning();
            hasTensioned = false;
        }
        if(gamepad2.dpad_down && gamepad2.y) {
            slideArm.stopTensioning();
            slideArm.loosenRightSlide = true;
            slideArm.loosenSlidesIndividual();
            hasDownTensioned = true;
        }
        if(gamepad2.dpad_up && gamepad2.y) {
            slideArm.stopTensioning();
            slideArm.loosenRightSlide = false;
            slideArm.loosenSlidesIndividual();
            hasDownTensioned = true;
        }
        if (gamepad2.y && !gamepad2.dpad_down && !gamepad2.dpad_up) {
            slideArm.loosenSlidesManual();
            hasDownTensioned = true;
        }
        if (!gamepad2.y && hasDownTensioned) {
            slideArm.stopTensioning();
            hasDownTensioned = false;
        }

        if (gamepad1.dpad_right && !teamChangeRequested) {
            teamChangeRequested = true;
            hiveColorChangeWasRequested = true;
            if (HIVE_COLOR == TeamColor.TEAM_BLUE) {
                HIVE_COLOR = TeamColor.TEAM_RED;
            }else{
                HIVE_COLOR = TeamColor.TEAM_BLUE;
            }
        }
        if (!gamepad1.dpad_right && teamChangeRequested){
            teamChangeRequested = false;
            teamChangeRequested = false;
        }
        if (gamepad1.guide) {
            restartTeleopDrive();
        }
        if (gamepad1.x){
            resetSpeciWasRequested = true;
            specimenArm.resetSpecimenWithoutLimits();
            resetSpeciTimer.reset();
        }
        if (!gamepad1.x && resetSpeciWasRequested) {
            resetSpeciWasRequested = false;
            speciResetNeeded = true;
            specimenArm.resetSpecimenEncoders();
        }
        if (resetSlideTimer.milliseconds() > resetSlideTimerThreshold && retractResetNeeded){
            slideArm.resetSlideEncoders();
            retractResetNeeded = false;
        }
        if (resetPivotTimer.milliseconds() > resetPivotTimerThreshold && pivotResetNeeded){
            slideArm.resetPivotEncoders();
            pivotResetNeeded = false;
        }
        if (resetSpeciTimer.milliseconds() > resetSpeciTimerThreshold && speciResetNeeded) {
            specimenArm.resetSpecimenEncoders();
            speciResetNeeded = false;
        }
        if (gamepad1.start){
            slideArm.zeroMotors();
            specimenArm.zeroMotors();
            motorsWereZeroed = true;
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
}
