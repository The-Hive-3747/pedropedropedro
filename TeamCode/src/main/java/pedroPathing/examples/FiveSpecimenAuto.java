package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.Constants;

import pedroPathing.TeleOpComp;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystem.IndicatorLight;
import pedroPathing.subsystem.OpModeTransfer;
import pedroPathing.subsystem.SlideArm;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.command.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.subsystem.SpecimenArm;

@Autonomous(name = "Five Specimen Auto")
public class FiveSpecimenAuto extends LinearOpMode {
    boolean teamChangeRequested = false;
    private ElapsedTime criticalLoop= new ElapsedTime();
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
    public static Pose startPose = new Pose(8,61.5, Math.toRadians(0));

    private Follower follower;

    public static PathChain preload() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(8, 61.5, Point.CARTESIAN),
                        new Point(37, 75, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pushSamples() {
        return new PathBuilder()
                .addPath(new BezierCurve( // after place, goes to first sample
                        new Point(37, 75, Point.CARTESIAN),
                        new Point(0.7, 43.8, Point.CARTESIAN),
                        new Point(54, 27, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // pushes first sample
                        new Point(54, 27, Point.CARTESIAN),
                        new Point(25, 20, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // goes to second sample
                        new Point(25, 20, Point.CARTESIAN),
                        new Point(48, 20, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve( // pushes second sample
                        new Point(48, 20, Point.CARTESIAN),
                        new Point(60, 10, Point.CARTESIAN),
                        new Point(25, 12, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine( // goes to third sample
                        new Point(25, 12, Point.CARTESIAN),
                        new Point(48, 14, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierCurve( // pushes third sample
                        new Point(48, 14, Point.CARTESIAN), // change this y if the robot is hitting the wall at the beginning of the path
                        new Point(70, 0.6, Point.CARTESIAN), // change this y if the robot is hitting the wall at the middle of the path
                        new Point(20, 2, Point.CARTESIAN))) // change this y if the robot is hitting the wall at the end of the path
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20,2,Point.CARTESIAN),
                        new Point(13,5, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain score1Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(13, 5, Point.CARTESIAN),
                        new Point(12, 70, Point.CARTESIAN),
                        new Point(39, 73.3, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

    }
    public static PathChain pickup2Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(39, 73.3, Point.CARTESIAN),
                        new Point(31, 73, Point.CARTESIAN),
                        new Point(20, 45, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20, 45, Point.CARTESIAN),
                        new Point(13, 42, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public static PathChain score2Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(13, 42, Point.CARTESIAN),
                        new Point(15, 60, Point.CARTESIAN),
                        new Point(38.3, 70.25, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pickup3Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(38.3, 70.25, Point.CARTESIAN),
                        new Point(31, 73, Point.CARTESIAN),
                        new Point(20, 45, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20, 45, Point.CARTESIAN),
                        new Point(14, 42, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain score3Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(14, 42, Point.CARTESIAN),
                        new Point(15, 70, Point.CARTESIAN),
                        new Point(38, 69.75, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain pickup4Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(38, 69.75, Point.CARTESIAN),
                        new Point(31, 73, Point.CARTESIAN),
                        new Point(20, 45, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(new BezierLine(
                        new Point(20, 45, Point.CARTESIAN),
                        new Point(14, 42, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain score4Specimen() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(14, 42, Point.CARTESIAN),
                        new Point(15, 77, Point.CARTESIAN),
                        new Point(38, 67.5, Point.CARTESIAN)
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
                follower.followPath(pushSamples());
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
                follower.followPath(pickup2Specimen());
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
                follower.followPath(pickup3Specimen());
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
                follower.followPath(pickup4Specimen());
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

        IndicatorLight leftLight = new IndicatorLight(hardwareMap, telemetry, "left_light");
        IndicatorLight rightLight = new IndicatorLight(hardwareMap, telemetry, "right_light");
        leftLight.setColor(IndicatorLight.COLOR_RED);
        rightLight.setColor(IndicatorLight.COLOR_RED);
        scheduler = CommandScheduler.getInstance();

        scheduler.schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                slideArm.new wristStow(),
                                specimenArm.new doAutoClawStateClose(),
                                specimenArm.new SpecimenArmEnter(),
                                this.new FollowPreload() // SCORE PRELOAD
                        ),
                        this.new RawDriveForward(),
                        specimenArm.new SpecimenArmScore(),
                        specimenArm.new doAutoClawStateOpen(),
                        specimenArm.new SpecimenArmCollect()
                                .alongWith(
                                        this.new FollowPushSamples()), // PUSH SAMPLES

                        specimenArm.new doAutoClawStateClose(),

                        specimenArm.new SpecimenArmEnter()
                                .alongWith(
                                        this.new FollowScore1Specimen() // SCORE FIRST SAMPLE
                                ),
                        this.new RawDriveForward(),
                        specimenArm.new SpecimenArmScore(),

                        specimenArm.new doAutoClawStateOpen(),
                        specimenArm.new SpecimenArmCollect()
                                .alongWith(
                                    this.new FollowPickup2Specimen()
                                ), // PICKUP SECOND SAMPLE
                        specimenArm.new doAutoClawStateClose(),

                        specimenArm.new SpecimenArmEnter()
                                .alongWith(
                                        this.new FollowScore2Specimen() // Score second sample
                                ),
                        this.new RawDriveForward(),
                        specimenArm.new SpecimenArmScore(),
                        specimenArm.new doAutoClawStateOpen(),
                        specimenArm.new SpecimenArmCollect()
                                .alongWith(
                                    this.new FollowPickup3Specimen()
                                ), // pickup third sample
                        specimenArm.new doAutoClawStateClose(),

                        specimenArm.new SpecimenArmEnter()
                                .alongWith(
                                        this.new FollowScore3Specimen() // score third sample
                                ),
                        this.new RawDriveForward(),
                        specimenArm.new SpecimenArmScore(),
                        specimenArm.new doAutoClawStateOpen(),
                        specimenArm.new SpecimenArmCollect()
                                .alongWith(

                                    this.new FollowPickup4Specimen()
                                ), // pickup fourth sample
                        specimenArm.new doAutoClawStateClose(),
                        new ParallelCommandGroup(
                                specimenArm.new SpecimenArmEnter(),
                                this.new FollowScore4Specimen(), // score fourth sample
                                slideArm.new wristReady()
                                ),
                        this.new RawDriveForward(),
                        specimenArm.new SpecimenArmScore(),
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

        waitForStart();

        slideArm.slideBrakes();
        leftLight.setColor(IndicatorLight.COLOR_BEECON);
        rightLight.setColor(IndicatorLight.COLOR_BEECON);
        while (opModeIsActive()) {
            follower.update();
            //slideArm.update();
            specimenArm.update();
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


        }

        OpModeTransfer.autoPose = follower.getPose();
    }
    }