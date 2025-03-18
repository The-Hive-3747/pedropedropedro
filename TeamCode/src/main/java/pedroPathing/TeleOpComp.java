package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
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
    public static int RUMBLE_HANG_TIME_OUT = 90;
    public static int RUMBLE_HANG_TIME2_OUT = 105;
    public static Stack<Double> battery_checker = new Stack<>();
    private boolean slideWasRetract = false;
    private boolean slideWasExtend = false;
    private boolean clawWasPushed = false;
    private boolean shoulderWasPushed = false;
    private boolean disableSpecimenArmWasPushed = false;
    private boolean releaseSlideArmHangWasPushed = false;
    private boolean configModeActivated = false;
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
    private double RAW_DRIVE_TIME = 200.0; //250.0;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private double DRIVE_POWER = 0.3;
    private SpecimenArm specimenArm = null;
    private ElapsedTime driveTime = new ElapsedTime();
    private ElapsedTime rumbleHangTimer = new ElapsedTime();
    private ElapsedTime shoulderTimer = new ElapsedTime();
    private SlideArm slideArm = null;
    private ElapsedTime pivotTimer = new ElapsedTime();
    private ElapsedTime wristTimer = new ElapsedTime();
    private ElapsedTime resetPivotTimer = new ElapsedTime();
    private ElapsedTime resetSlideTimer = new ElapsedTime();
    private ElapsedTime resetSpeciTimer = new ElapsedTime();
    private ElapsedTime criticalLoopTimer = new ElapsedTime();
    private ElapsedTime slowDownTimer = new ElapsedTime();
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
    private double maxLoopTime = 0.0;
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);
    private boolean resetMotors = false;
    @Override
    public void runOpMode() throws InterruptedException {

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
        }
        waitForStart();

        rumbleHangTimer.reset();
        slideArm.setWristToReady();
        follower.startTeleopDrive();


        criticalLoopTimer.reset();
        while (opModeIsActive()) {
            if (configModeActivated){
                runConfigMode();
            }else {

                if (gamepad2.left_stick_button && !disableSpecimenArmWasPushed) {
                    specimenArm.disableSpecimenArm();
                    disableSpecimenArmWasPushed = true;
                } else if (!gamepad2.left_stick_button && disableSpecimenArmWasPushed){
                    disableSpecimenArmWasPushed = false;
                }
                /*if (gamepad2.left_stick_x > STICK_X_RIGHT) {
                    slideArm.moveWristRight();
                }
                if (gamepad2.left_stick_x < STICK_X_LEFT) {
                    slideArm.moveWristLeft();
                }*/

                /*if (gamepad1.x && shoulderTimer.milliseconds() > shoulderTimerThreshold) {
                    telemetry.addData("Status", "Shoulder Moved");
                    shoulderTimer.reset();
                    specimenArm.goToNextSpecimenState();
                }*/
                /*if (rumbleHangTimer.seconds() >= RUMBLE_HANG_TIME_OUT && !rumble1Fired) {
                   rumble1Fired = true;
                   gamepad1.rumble(300);
                   gamepad2.rumble(300);
                }else if (rumbleHangTimer.seconds() >= RUMBLE_HANG_TIME2_OUT && !rumble2Fired){
                    rumble2Fired = true;
                    gamepad1.rumble(300);
                    gamepad2.rumble(300);
                }*/
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

                if (gamepad2.b && !pivotHangRequested) {
                    slideArm.pivotHang();
                    specimenArm.clawStateClose();
                    //specimenArm.goToSpecimenCollect();
                    slideArm.setWristToStow();
                    pivotHangRequested = true;
                }
                if (!gamepad2.b && pivotHangRequested) {
                    pivotHangRequested = false;
                }
                if (gamepad2.a && !aWasPushed) {
                    aWasPushed = true;
                    slideArm.hangHold();
                }
                if (!gamepad2.a && aWasPushed){
                    aWasPushed = false;
                    //slideArm.stopHang();
                }
                if (gamepad2.dpad_up && !sitWasPushed){
                    sitWasPushed = true;
                    slideArm.sit();
                }
                if (!gamepad2.dpad_up && sitWasPushed){
                    sitWasPushed = false;
                }
                if (gamepad1.y && !clawWasPushed) {
                    specimenArm.nextClawState();
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
                if (gamepad2.dpad_right) {
                    slideWasExtend = true;
                    slideArm.slideUpOneStep();
                }
                if (slideWasExtend && !gamepad2.dpad_right) {
                    slideWasExtend = false;
                    slideArm.stopSliding();
                }
                if (!clawSensorRan && !clawSensorWasPressed && specimenArm.clawSensor.isPressed()) {
                    specimenArm.clawSensorGrab();
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
                if (gamepad2.left_bumper) {
                    isIntaking = true;
                    //leftLight.setColor(IndicatorLight.COLOR_BEECON);
                    slideArm.isKeeperBlock();
                    slideArm.activateIntakeWithSensor();
                }

                    //leftLight.setColor(leftLight.COLOR_SAGE);
                    //rightLight.setColor(rightLight.COLOR_SAGE);

                if (!gamepad2.right_bumper && !gamepad2.left_bumper && intakeRequested) {
                    isIntaking = false;
                    slideArm.stopIntake();
                    intakeRequested = false;
                    //leftLight.setColor(leftLight.COLOR_BEECON);
                    //rightLight.setColor(rightLight.COLOR_BEECON);
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
                    //leftLight.setColor(leftLight.COLOR_BEECON);
                    //rightLight.setColor(rightLight.COLOR_BEECON);
                }
                if (gamepad2.left_trigger <= LEFT_TRIGGER_THRESHOLD && wristChangeRequested) {
                    wristChangeRequested = false;
                }
                /*
                if (gamepad1.a) {
                    //TODO Use Normal hang hold
                    slideArm.firstHangHold();
                }
                if (gamepad1.dpad_right){
                    //TODO Use Normal Pivot
                    slideArm.firstPivot();
                }
                */
                if (gamepad1.b) {
                    follower.setPose(startPose);
                }
            }
            if (gamepad1.dpad_up && !configWasRequested) {
                configModeActivated = false;
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
            /*drive.setDrivePowers(new PoseVelocity2d(
                    Rotation2d.fromDouble(
                            -drive.pose.heading.toDouble()).times(
                            new Vector2d(
                                    -leftJoyStickSpeedY*speedMultiplier,
                                    -leftJoyStickSpeedX*speedMultiplier)),
                    -gamepad1.right_stick_x*speedMultiplier +
                            (strafeBias*(-leftJoyStickSpeedX*speedMultiplier) +
                                    (strafeBiasY*(-leftJoyStickSpeedX*speedMultiplier)))
            ));
            drive.updatePoseEstimate();*/
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
            else if (slideArm.hangRequested){
                leftLight.setColor(IndicatorLight.COLOR_VIOLET);
                rightLight.setColor(IndicatorLight.COLOR_VIOLET);
            }else if (isIntaking && slideArm.isWristGather()){
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
            /*telemetry.addData("Hit gamepad1.X to move shoulder","");
            telemetry.addData("Hit gamepad1.Y to move claw","");
            telemetry.addData("Hit gamepad2.B to pivot for hang","");
            telemetry.addData("Hit gamepad2.A to hang from slides","");
            telemetry.addData("Hit gamepad1.left_bumper to change speed level","");
            telemetry.addData("Hit gamepad2.left_bumper to activate the intake with the sensor","");
            telemetry.addData("Hit gamepad2.right_trigger to score sample in bucket","");
            telemetry.addData("Hit gamepad2.left_trigger to change wrist pivot","");
            telemetry.addData("Hit gamepad2.right_bumper to activate the intake without the sensor","");
            telemetry.addData("Hit gamepad2.Y to pivot slide arm UP", "");
            telemetry.addData("Hit gamepad2.X to pivot slide arm DOWN", "");
            telemetry.addData("Hit gamepad2.dpad RIGHT to extend the slide arm - WITH LIMITER","");
            telemetry.addData("Hit gamepad2.dpad LEFT to retract the slide arm - WITH LIMITER","");
            telemetry.addData("Hit gamepad2 Right Stick Button to STOW specimen arm","");
            //telemetry.addData("Hit the gamepad1.BACK button to slow release hang", "");
            telemetry.addData("Hit gamepad1.B to reset pedro orientation.", "");
            telemetry.addData("Hit gamepad1.back to slowly come down from hang.", "");
            telemetry.addData("Hit gamepad1 dpad UP to go into CONFIGURE", "");
            telemetry.addData("--------------------------------CONFIG MODE", "");
            telemetry.addData("Hit gamepad 1 dpad RIGHT to change TEAM color", "");*/
            telemetry.update();
            hiveColorChangeWasRequested = false;

            // TODO: IF FTC DASHBOARD NEEDED: UNCOMMENT TELEMETRYPACKET & FTCDASHBOARD

            //TelemetryPacket packet = new TelemetryPacket();

            //packet.fieldOverlay().setStroke("#3F51B5");
            //Drawing.drawRobot(packet.fieldOverlay(), drive.pose);

            //FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
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
        public void stopDriveForward() {
                leftBack.setPower(0);
                rightBack.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(0);
                follower.startTeleopDrive();
    }


    public void runConfigMode(){
        if (gamepad2.b) {
            pivotResetWasRequested = true;
            slideArm.pivotDownWithoutLimits();
            resetPivotTimer.reset();
        }
        if (!gamepad2.b && pivotResetWasRequested) {
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
}
