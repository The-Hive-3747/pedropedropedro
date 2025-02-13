package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.subsystem.IndicatorLight;
import pedroPathing.subsystem.OpModeTransfer;
import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;

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
    private boolean slideWasRetract = false;
    private boolean slideWasExtend = false;
    private boolean clawWasPushed = false;
    private boolean shoulderWasPushed = false;
    private boolean configModeActivated = false;
    private boolean configWasRequested = false;
    private boolean pivotResetWasRequested = false;
    private boolean retractResetWasRequested = false;
    private boolean teamChangeRequested = false;
    private boolean resetSpeciWasRequested = false;
    private boolean retractResetNeeded = false;
    private boolean pivotResetNeeded = false;
    private boolean speciResetNeeded = false;
    private SpecimenArm specimenArm = null;
    private ElapsedTime shoulderTimer = new ElapsedTime();
    private SlideArm slideArm = null;
    private ElapsedTime pivotTimer = new ElapsedTime();
    private ElapsedTime wristTimer = new ElapsedTime();
    private ElapsedTime resetPivotTimer = new ElapsedTime();
    private ElapsedTime resetSlideTimer = new ElapsedTime();
    private ElapsedTime resetSpeciTimer = new ElapsedTime();
    private ElapsedTime criticalLoopTimer = new ElapsedTime();
    //private static Pose2d RESET_POSE = new Pose2d(0.0, 0.0, 0.0);
    private static double pivotTimerThreshold = 1000.0;
    private static double wristTimerThreshold = 750.0;
    private static double shoulderTimerThreshold = 500.0;
    private static double resetPivotTimerThreshold = 1000.0;
    private static double resetSlideTimerThreshold = 1000.0;
    private static double resetSpeciTimerThreshold = 1000.0;
    public static float LEFT_TRIGGER_THRESHOLD = 0.7f;
    public static float RIGHT_TRIGGER_THRESHOLD = 0.7f;
    private static double STICK_X_LEFT = -0.6;
    private static double STICK_X_RIGHT = 0.6;
    private static double NORMAL_SPEED = 0.8;
    private static double LUDICROUS_SPEED = 1.0;
    private static double SLOW_SPEED = 0.5;
    private double maxLoopTime = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {

        HIVE_COLOR = OpModeTransfer.autoColor;
        IndicatorLight leftLight = new IndicatorLight(hardwareMap, telemetry, "left_light");
        IndicatorLight rightLight = new IndicatorLight(hardwareMap, telemetry, "right_light");

        leftLight.setColor(IndicatorLight.COLOR_RED);
        rightLight.setColor(IndicatorLight.COLOR_RED);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

        specimenArm = new SpecimenArm(hardwareMap, telemetry);
        slideArm = new SlideArm(hardwareMap, telemetry, false);
        //leftLight.setColor(IndicatorLight.COLOR_GREEN);
        //rightLight.setColor(IndicatorLight.COLOR_GREEN);
        if (HIVE_COLOR == TeamColor.TEAM_BLUE) {
            HIVE_COLOR = TeamColor.TEAM_RED;
        }else{
            HIVE_COLOR = TeamColor.TEAM_BLUE;
        }
        waitForStart();

        slideArm.setWristToReady();

        leftLight.setColor(IndicatorLight.COLOR_BEECON);
        rightLight.setColor(IndicatorLight.COLOR_BEECON);

        criticalLoopTimer.reset();
        while (opModeIsActive()) {
            if (configModeActivated){
                runConfigMode();
            }else {

                if (gamepad2.left_stick_button) {
                    specimenArm.disableSpecimenArm();
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
                if (gamepad1.x && !shoulderWasPushed) {
                    specimenArm.goToNextSpecimenState();
                    shoulderWasPushed = true;
                } else if (!gamepad1.x && shoulderWasPushed) {
                    shoulderWasPushed = false;
                }

                if (gamepad2.b) {
                    slideArm.pivotHang();
                }
                if (gamepad2.a) {
                    slideArm.hangHold();
                }
                if (gamepad1.y && !clawWasPushed) {
                    specimenArm.nextClawState();
                    clawWasPushed = true;
                } else if (!gamepad1.y && clawWasPushed) {
                    clawWasPushed = false;
                }
                if (gamepad1.back) {
                    slideArm.hangRelease();
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
                if (gamepad2.y && pivotTimer.milliseconds() > pivotTimerThreshold) {
                    telemetry.addData("Status", "Pivoting Up");
                    pivotTimer.reset();
                    specimenArm.clawStateClose();
                    slideArm.pivotUp();
                }
                if (gamepad2.x && pivotTimer.milliseconds() > pivotTimerThreshold) {
                    telemetry.addData("Status", "Pivoting Down");
                    pivotTimer.reset();
                    specimenArm.clawStateClose();
                    slideArm.pivotDown();
                }
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    slideArm.makeLimp();
                    specimenArm.makeLimp();
                }
                if (gamepad2.right_bumper) {
                    slideArm.activateIntakeWithoutSensor();
                }
                if (gamepad2.left_bumper) {
                    slideArm.isKeeperBlock();
                    slideArm.activateIntakeWithSensor();
                }
                if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                    slideArm.stopIntake();
                }
                if (gamepad2.right_trigger > RIGHT_TRIGGER_THRESHOLD) {
                    slideArm.reverseIntakeWithoutSensor();
                }
                if (gamepad2.start) {
                    slideArm.slowMoveMotors(true);
                }
                if (gamepad2.left_trigger > LEFT_TRIGGER_THRESHOLD && wristTimer.milliseconds() > wristTimerThreshold) {
                    wristTimer.reset();
                    slideArm.nextWristPosition();
                }
                if (gamepad1.b) {
                    //drive.pose = new Pose2d(0.0, 0.0, 0.0);
                }
            }
            if (gamepad1.dpad_up && !configWasRequested) {
                configModeActivated = !configModeActivated;
                configWasRequested = true;
            } else if (!gamepad1.dpad_up && configWasRequested) {
                configWasRequested = false;
            }
            speedMultiplier = NORMAL_SPEED;
            if (gamepad1.left_bumper){
                speedMultiplier = SLOW_SPEED;
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
                    rightLight.setColor(IndicatorLight.COLOR_SAGE);
                }else{
                    leftLight.setColor(IndicatorLight.COLOR_RED);
                    rightLight.setColor(IndicatorLight.COLOR_SAGE);
                }
            }
            else if (slideArm.hangRequested){
                leftLight.setColor(IndicatorLight.COLOR_VIOLET);
                rightLight.setColor(IndicatorLight.COLOR_VIOLET);
            }else{
                leftLight.setColor(IndicatorLight.COLOR_BEECON);
                rightLight.setColor(IndicatorLight.COLOR_BEECON);
            }

            slideArm.update();
            specimenArm.update();
            if (maxLoopTime < criticalLoopTimer.milliseconds()){
                maxLoopTime = criticalLoopTimer.milliseconds();
            }
            telemetry.addData("Team Color", HIVE_COLOR);
            telemetry.addData("Config Mode Activated", configModeActivated);
            telemetry.addData("Critical Loop MS", criticalLoopTimer.milliseconds());
            criticalLoopTimer.reset();
            telemetry.addData("Max Critical Loop Time", maxLoopTime);
            telemetry.addData("Intake Distance", slideArm.getIntakeDistaceCM());
            telemetry.addData("Intake Color:", slideArm.detectColor());
            //telemetry.addData("x", drive.pose.position.x);
            //telemetry.addData("y", drive.pose.position.y);
            //telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("Intake Status", slideArm.getIntakeStatus());
            telemetry.addData("Hang Requested", slideArm.hangRequested);
            telemetry.addData("Wrist Status", slideArm.getWristPosition());
            slideArm.addSlideTelemetry();
            telemetry.addData("Right Shoulder Ticks", specimenArm.rightShoulder.getCurrentPosition());
            telemetry.addData("Hit gamepad2.X to move shoulder","");
            telemetry.addData("Hit gamepad2.Y to move claw","");
            telemetry.addData("Hit gamepad2.A to pivot for hang","");
            telemetry.addData("Hit gamepad2.B to hang from slides","");
            telemetry.addData("Hit gamepad1.left_bumper to change speed level","");
            telemetry.addData("Hit gamepad2.left_bumper to activate the intake with the sensor","");
            telemetry.addData("Hit gamepad2.right_trigger to score sample in bucket","");
            telemetry.addData("Hit gamepad2.left_trigger to force wrist to front","");
            telemetry.addData("Hit gamepad2.right_bumper to activate the intake without the sensor","");
            telemetry.addData("Hit gamepad2.dpad UP to pivot slide arm UP", "");
            telemetry.addData("Hit gamepad2.dpad DOWN to pivot slide arm DOWN", "");
            telemetry.addData("Hit gamepad2.dpad RIGHT to extend the slide arm - WITH LIMITER","");
            telemetry.addData("Hit gamepad2.dpad LEFT to retract the slide arm - WITH LIMITER","");
            telemetry.addData("Hit gamepad2 Right Stick Button to STOW specimen arm","");
            telemetry.addData("Hit the BACK button to cut power to slide arm", "");
            telemetry.addData("Cutting power to slide arm will cause it to DROP", "");
            telemetry.addData("Hit gamepad1.Y to reset roadrunner orientation.", "");
            telemetry.addData("Hit gamepad1.back to slowly come down from hang.", "");
            telemetry.addData("Hit gamepad1 dpad UP to go into CONFIGURE", "");
            telemetry.addData("--------------------------------CONFIG MODE", "");
            telemetry.addData("Hit gamepad 1 dpad RIGHT to change TEAM color", "");
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
    public void runConfigMode(){
        if (gamepad2.dpad_down) {
            pivotResetWasRequested = true;
            slideArm.pivotDownWithoutLimits();
            resetPivotTimer.reset();
        }
        if (!gamepad2.dpad_down && pivotResetWasRequested) {
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
            if (HIVE_COLOR == TeamColor.TEAM_BLUE) {
                HIVE_COLOR = TeamColor.TEAM_RED;
            }else{
                HIVE_COLOR = TeamColor.TEAM_BLUE;
            }
        }
        if (!gamepad1.dpad_right && teamChangeRequested){
            teamChangeRequested = false;
        }
        if (gamepad2.x){
            resetSpeciWasRequested = true;
            specimenArm.resetSpecimenWithoutLimits();
            resetSpeciTimer.reset();
        }
        if (!gamepad2.x && resetSpeciWasRequested) {
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
    }
}
