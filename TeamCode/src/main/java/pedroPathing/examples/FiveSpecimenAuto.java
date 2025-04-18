package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;

import pedroPathing.TeleOpComp;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystem.IndicatorLight;
import pedroPathing.subsystem.LimelightVision;
import pedroPathing.subsystem.OpModeTransfer;
import pedroPathing.subsystem.SlideArm;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.subsystem.SpecimenArm;

@Autonomous(name = "Five Specimen Auto")
public class FiveSpecimenAuto extends LinearOpMode {
    boolean teamChangeRequested = false;
    private ElapsedTime criticalLoop= new ElapsedTime();
    private ElapsedTime touchLightTimer = new ElapsedTime();
    TeleOpComp.TeamColor HIVE_COLOR = TeleOpComp.TeamColor.TEAM_BLUE;
    public String pathState = null;
    private SlideArm slideArm = null;
    private SpecimenArm specimenArm = null;
    private CommandScheduler scheduler = null;
    private double RAW_DRIVE_TIME = 200.0; //250.0;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private double DRIVE_POWER = 0.3;
    private boolean isPreload = false;
    private boolean touchSensorHit = false;
    private Pose limelightPose = null;
    public static Pose startPose = new Pose(8,61.5, Math.toRadians(0));
    private LimelightVision limelightVision;
    private Telemetry telemetryA;
    private static TelemetryPacket packet;
    private double PUSH_SAMPLES_X_BARRIER = 25;
    private int TOUCH_SENSOR_TIMER_FLASH_TIME = 1000;
    private long TIME_CLAW_WAIT = 200;
    private boolean backupUsed = false;

    private Follower follower;

    public static PathChain preload() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(8, 61.5, Point.CARTESIAN),
                        new Point(38.5, 83, Point.CARTESIAN))) //75
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(10))
                .build();
    }
    public static PathChain pushSamples() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(38.5, 83, Point.CARTESIAN),
                        //new Point(32.5, 10, Point.CARTESIAN),
                        new Point(28,43, Point.CARTESIAN)))//25, 42
                .setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(0))
                .addPath(new BezierLine( // after place, goes to first sample
                        new Point(28, 43, Point.CARTESIAN), //25, 42
                        //new Point(0, 60.0, Point.CARTESIAN), //x:0.7 y:43.8
                        new Point(50.0, 30.5, Point.CARTESIAN))) //x:53.5, y:28
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve( // pushes first sample
                        new Point(50.0, 30.5, Point.CARTESIAN),//x:53.5, y:28
                        new Point(80, 25, Point.CARTESIAN),
                        new Point(25, 23, Point.CARTESIAN))) //21
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // goes to second sample
                        new Point(25, 23, Point.CARTESIAN),
                        new Point(46.5, 23, Point.CARTESIAN))) //22 //x:47.5 royd
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve( // pushes second sample
                        new Point(47.5, 23, Point.CARTESIAN),
                        new Point(60, 9, Point.CARTESIAN),
                        new Point(25, 11, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // goes to third sample
                        new Point(25, 11, Point.CARTESIAN),
                        new Point(44.5, 15, Point.CARTESIAN))) //y13.5 / x:48, y:10 //x:47 royd
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve( // pushes third sample
                        new Point(44.5, 15, Point.CARTESIAN), // y13.5 / x:48, y:10// change this y if the robot is hitting the wall at the beginning of the path
                         new Point(70, 10, Point.CARTESIAN), //6.2 /  0.6 / change this y if the robot is hitting the wall at the middle of the path
                        new Point(55, 8, Point.CARTESIAN))) // change this y if the robot is hitting the wall at the end of the path
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(55,8,Point.CARTESIAN),
                        new Point(8,6.5, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain score1Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(8, 6.5, Point.CARTESIAN),
                        new Point(13.5, 70, Point.CARTESIAN),
                        new Point(39, 76.5, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                //.setPathEndTValueConstraint(0.99)
                .build();

    }
    public static PathChain pickup2Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(39, 76.5, Point.CARTESIAN),
                        new Point(31, 73, Point.CARTESIAN),
                        new Point(20, 45, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20, 45, Point.CARTESIAN),
                        new Point(15.5, 42, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                //.setZeroPowerAccelerationMultiplier(2)
                .build();
    }

    public static PathChain score2Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(15.5, 42, Point.CARTESIAN),
                        new Point(15, 60, Point.CARTESIAN),
                        new Point(39, 74.5, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pickup3Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(39, 74.5, Point.CARTESIAN), // 38.3
                        new Point(31, 73, Point.CARTESIAN),
                        new Point(20, 45, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20, 45, Point.CARTESIAN),
                        new Point(15.5, 42, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain score3Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(15.5, 42, Point.CARTESIAN),
                        new Point(15, 70, Point.CARTESIAN),
                        new Point(39, 72.5, Point.CARTESIAN) // 38
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pickup4Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(39, 72.5, Point.CARTESIAN),
                        new Point(31, 73, Point.CARTESIAN),
                        new Point(20, 45, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20, 45, Point.CARTESIAN),
                        new Point(15.5, 42, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain score4Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(15.5, 42, Point.CARTESIAN),
                        new Point(15, 80, Point.CARTESIAN),
                        new Point(39, 70.5, Point.CARTESIAN) // 68.5 y70 x38
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public class RawDriveForward extends CommandBase {
        ElapsedTime driveTime = null;
        boolean firstTime = true;
        boolean isDone = false;
        public RawDriveForward() {}
        @Override
        public void initialize() {
            driveTime = new ElapsedTime();
            firstTime= true;
            isDone = false;
        }
        @Override
        public void execute() {
            pathState = "rawDrive";
            if (firstTime) {
                driveTime.reset();
                firstTime = false;
                follower.breakFollowing();
                leftBack.setPower(DRIVE_POWER);
                rightBack.setPower(DRIVE_POWER);
                leftFront.setPower(DRIVE_POWER);
                rightFront.setPower(DRIVE_POWER);
                leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                /*if (isPreload) {
                    RAW_DRIVE_TIME = 400;
                } else {
                    RAW_DRIVE_TIME = 150;
                }*/
            }

            if (driveTime.milliseconds()>RAW_DRIVE_TIME) {
                leftBack.setPower(0);
                rightBack.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(0);
                isDone=true;
            }
        }
        @Override
        public boolean isFinished() {
            return isDone;
        }
    }

    public class FollowPreload extends CommandBase {
        boolean followerStarted = false;
        public FollowPreload() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(preload());
                isPreload = true;
                pathState = "score preload";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    public class FollowPushSamples extends CommandBase {
        boolean followerStarted = false;
        public FollowPushSamples() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                isPreload = false;
                follower.followPath(pushSamples(), false);
                pathState = "pushSamples";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowScore1Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowScore1Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score1Specimen());
                pathState = "score 2nd speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowPickup2Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowPickup2Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(pickup2Specimen());//, 0.8, true);
                pathState = "pickup 3rd speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    public class FollowScore2Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowScore2Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score2Specimen());
                pathState = "score 3rd speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowPickup3Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowPickup3Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(pickup3Specimen());//, 0.8, true);
                pathState = "pickup 4th speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowScore3Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowScore3Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score3Specimen());
                pathState = "score 4th speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowPickup4Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowPickup4Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(pickup4Specimen());//, 0.8, true);
                pathState = "pickup 5th speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowScore4Specimen extends CommandBase {
        boolean followerStarted = false;
        public FollowScore4Specimen() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score4Specimen());
                pathState = "score 5th speci";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class closeClawUsingSensor extends CommandBase {
        boolean isPressed = false;
        boolean isDone = false;
        boolean isClawClose = false;

        public closeClawUsingSensor() {}
        @Override
        public void initialize() {
            isPressed = false;
            isDone = false;
            isClawClose = false;
        }

        @Override
        public void execute() {
            if (specimenArm.clawSensor.isPressed() && !isPressed && follower.getPose().getX() < PUSH_SAMPLES_X_BARRIER) {
                isPressed = true;
                touchSensorHit = true;
                touchLightTimer.reset();
                //clawStateClose();
            }
            if (isPressed && !isClawClose) {
                isClawClose = true;
                specimenArm.specimenClawTime.reset();
                specimenArm.clawStateClose();
            }
            if (isClawClose && (specimenArm.specimenClawTime.milliseconds() > specimenArm.SPECIMENCLAW_CLOSE_TIME)) {
                isDone = true;
            }
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
        @Override
        public void end(boolean interrupted) {
            if(interrupted) {
                touchLightTimer.reset();
                backupUsed = true;
                specimenArm.clawStateClose();
            }
        }
    }
    @Override
    public void runOpMode() {
        //Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        slideArm = new SlideArm(hardwareMap, telemetry, true);
        specimenArm = new SpecimenArm(hardwareMap, telemetry, true);
        leftFront = hardwareMap.get(DcMotor.class, FollowerConstants.leftFrontMotorName);
        rightFront = hardwareMap.get(DcMotor.class,FollowerConstants.rightFrontMotorName);
        leftBack = hardwareMap.get(DcMotor.class,FollowerConstants.leftRearMotorName);
        rightBack = hardwareMap.get(DcMotor.class,FollowerConstants.rightRearMotorName);
        limelightVision = new LimelightVision(hardwareMap);
        packet = new TelemetryPacket();
        IndicatorLight leftLight = new IndicatorLight(hardwareMap, telemetry, "left_light");
        IndicatorLight rightLight = new IndicatorLight(hardwareMap, telemetry, "right_light");
        leftLight.setColor(IndicatorLight.COLOR_RED);
        rightLight.setColor(IndicatorLight.COLOR_RED);
        scheduler = CommandScheduler.getInstance();

        scheduler.schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                slideArm.new PivotSlideArmUp(),
                                slideArm.new wristStow(),
                                specimenArm.new doAutoClawStateClose(),
                                specimenArm.new SpecimenArmEnter(),
                                this.new FollowPreload() // SCORE PRELOAD
                        ),
                        new ParallelCommandGroup(
                                this.new RawDriveForward(),
                                specimenArm.new SpecimenArmScore()
                        ),

                        new ParallelCommandGroup(
                                specimenArm.new doAutoClawStateOpen(),
                            specimenArm.new SpecimenArmCollect(),
                                slideArm.new PivotSlideArmDown(),
                                new ParallelRaceGroup(
                                    this.new FollowPushSamples(),
                                    this.new closeClawUsingSensor()
                                ).withTimeout(10250) //10250) // TODO: make this smart
                        ),
                        new WaitCommand(TIME_CLAW_WAIT),
                        // PUSH SAMPLES

                        //specimenArm.new doAutoClawStateClose(),

                        specimenArm.new SpecimenArmEnter()
                                .alongWith(
                                        this.new FollowScore1Specimen() // SCORE FIRST SAMPLE
                                ),
                        new ParallelCommandGroup(
                                this.new RawDriveForward(),
                            specimenArm.new SpecimenArmScore()
                        ),


                        new ParallelCommandGroup(
                                specimenArm.new doAutoClawStateOpen(),
                            specimenArm.new SpecimenArmCollect(),
                                new ParallelRaceGroup(
                                    this.new FollowPickup2Specimen(), // PICKUP SECOND SAMPLE
                                    this.new closeClawUsingSensor()
                                )
                        ),
                        new WaitCommand(TIME_CLAW_WAIT),
                        //specimenArm.new doAutoClawStateClose(),

                        specimenArm.new SpecimenArmEnter()
                                .alongWith(
                                        this.new FollowScore2Specimen() // Score second sample
                                ),
                        new ParallelCommandGroup(
                                this.new RawDriveForward(),
                                specimenArm.new SpecimenArmScore()
                        ),

                        new ParallelCommandGroup(
                                specimenArm.new doAutoClawStateOpen(),
                                specimenArm.new SpecimenArmCollect(),
                                new ParallelRaceGroup(
                                        this.new FollowPickup3Specimen(), // PICKUP third SAMPLE
                                        this.new closeClawUsingSensor()
                                )
                        ),
                        new WaitCommand(TIME_CLAW_WAIT),
                        //specimenArm.new doAutoClawStateClose(),

                        specimenArm.new SpecimenArmEnter()
                                .alongWith(
                                        this.new FollowScore3Specimen() // score third sample
                                ),
                        new ParallelCommandGroup(
                                this.new RawDriveForward(),
                                specimenArm.new SpecimenArmScore()
                        ),

                        new ParallelCommandGroup(
                                specimenArm.new doAutoClawStateOpen(),
                                specimenArm.new SpecimenArmCollect(),
                                new ParallelRaceGroup(
                                        this.new FollowPickup4Specimen(), // PICKUP fourth SAMPLE
                                        this.new closeClawUsingSensor()
                                )
                        ),
                        new WaitCommand(TIME_CLAW_WAIT),
                        //specimenArm.new doAutoClawStateClose(),
                        new ParallelCommandGroup(
                                specimenArm.new SpecimenArmEnter(),
                                //slideArm.new PivotSlideArmUp(),
                                this.new FollowScore4Specimen(), // score fourth sample
                                slideArm.new wristReady()
                                ),
                        new ParallelCommandGroup(
                                this.new RawDriveForward(),
                                specimenArm.new SpecimenArmScore()
                        ),
                        specimenArm.new doAutoClawStateOpen(),
                        specimenArm.new SpecimenArmCollect()
                )
        );
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_right && !teamChangeRequested) {
                teamChangeRequested = true;
                if (HIVE_COLOR == TeleOpComp.TeamColor.TEAM_BLUE) {
                    HIVE_COLOR = TeleOpComp.TeamColor.TEAM_RED;
                    OpModeTransfer.autoColor = TeleOpComp.TeamColor.TEAM_RED;
                } else {
                    HIVE_COLOR = TeleOpComp.TeamColor.TEAM_BLUE;
                    OpModeTransfer.autoColor = TeleOpComp.TeamColor.TEAM_BLUE;
                }
            }
            if (!gamepad1.dpad_right && teamChangeRequested) {
                teamChangeRequested = false;
            }
            if (HIVE_COLOR == TeleOpComp.TeamColor.TEAM_BLUE) {
                leftLight.setColor(IndicatorLight.COLOR_BLUE);
                rightLight.setColor(IndicatorLight.COLOR_BEECON);
            } else {
                leftLight.setColor(IndicatorLight.COLOR_RED);
                rightLight.setColor(IndicatorLight.COLOR_BEECON);
            }
        }
        /*telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("wow");
        telemetryA.update();*/
        slideArm.derailShiftDown();
        waitForStart();
        slideArm.derailRelax();

        slideArm.slideBrakes();

        while (opModeIsActive()) {
            follower.update();
            //slideArm.update();
            specimenArm.update();

            /*limelightPose = limelightVision.getRobotPose(follower.getPose().getHeading());
            if (limelightPose.getX() != 0 && limelightPose.getY() != 0){
                follower.setPose(limelightPose);
            }*/
            // Feedback to Driver Hub
            telemetry.addData("speci arm state", specimenArm.getShoulderState());
            telemetry.addData("Path State",  pathState);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.addData("Critical Loop (MS)",criticalLoop.milliseconds());
            telemetry.update();
            scheduler.run();
            criticalLoop.reset();
            Drawing.drawDebug(follower);

            if (touchSensorHit) {
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
            else {
                leftLight.setColor(IndicatorLight.COLOR_BEECON);
                rightLight.setColor(IndicatorLight.COLOR_BEECON);
            }



        }

        OpModeTransfer.autoPose = follower.getPose();
        OpModeTransfer.autoColor = HIVE_COLOR;
    }
    }