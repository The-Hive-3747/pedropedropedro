package pedroPathing.examples;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.TeleOpComp;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystem.IndicatorLight;
import pedroPathing.subsystem.OpModeTransfer;
import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;
@Disabled
@Autonomous(name = "SampleAutoSlide")
public class SampleAutoSlide extends LinearOpMode {
    boolean teamChangeRequested = false;
    TeleOpComp.TeamColor HIVE_COLOR = TeleOpComp.TeamColor.TEAM_BLUE;
    public String pathState = null;
    private SlideArm slideArm = null;
    private SpecimenArm specimenArm = null;
    private CommandScheduler scheduler = null;
    private double RAW_DRIVE_TIME = 500.0; //250.0;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private double DRIVE_POWER = 0.3;
    public static Pose startPose = new Pose(8,111, java.lang.Math.toRadians(0));
    public static Point INTAKE_SAMPLE_ONE_POINT = new Point(12, 123.5, Point.CARTESIAN);
    public static double INTAKE_SAMPLE_ONE_HEADING = Math.toRadians(8.5);
    public static Point SCORE_SAMPLE_POINT = new Point(10, 122.5, Point.CARTESIAN);
    public static double SCORE_SAMPLE_HEADING = Math.toRadians(-55);
    public static long AUTO_FIRST_INTAKE_PICKUP_THRESHOLD = 2000;

    private Follower follower;

    public static PathChain preload() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(8, 111, Point.CARTESIAN),
                        new Point(13, 114, Point.CARTESIAN),
                        new Point(7, 123.5, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(java.lang.Math.toRadians(0), java.lang.Math.toRadians(-45))
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
                follower.followPath(preload(),0.8, true);
                pathState = "score preload sample";

                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class IntakeSample1 extends CommandBase {
        boolean followerStarted = false;
        public IntakeSample1() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.turnTo(Math.toRadians(INTAKE_SAMPLE_ONE_HEADING));
                follower.holdPoint(INTAKE_SAMPLE_ONE_POINT, INTAKE_SAMPLE_ONE_HEADING);
                pathState = "intake first sample";

                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class IntakeHoldSample1 extends CommandBase {
        boolean followerStarted = false;
        public IntakeHoldSample1() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {

                follower.holdPoint(INTAKE_SAMPLE_ONE_POINT, INTAKE_SAMPLE_ONE_HEADING);
                pathState = "holding first sample";

                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return false;

        }
    }


    public class ScoreSample1 extends CommandBase {
        boolean followerStarted = false;
        public ScoreSample1() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.turnTo(Math.toRadians(SCORE_SAMPLE_HEADING));
                follower.holdPoint(SCORE_SAMPLE_POINT, SCORE_SAMPLE_HEADING);
                pathState = "intake first sample";

                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class ScoreSample1Hold extends CommandBase {
        boolean followerStarted = false;
        public ScoreSample1Hold() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.holdPoint(SCORE_SAMPLE_POINT, SCORE_SAMPLE_HEADING);
                pathState = "intake first sample";

                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return false;
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


        //waitForStart();
        scheduler = CommandScheduler.getInstance();
        //slideArm.slideBrakes();
        //leftLight.setColor(IndicatorLight.COLOR_BEECON);
        //rightLight.setColor(IndicatorLight.COLOR_BEECON);
        scheduler.schedule(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            this.new FollowPreload(),
                            slideArm.new PivotSlideArmUp(),
                            slideArm.new wristReady()
                            ),
                            slideArm.new ExtendSlideArm(),
                            slideArm.new wristScore(),
                            //new WaitCommand(500),
                            slideArm.new IntakeScore(),
                            slideArm.new stopIntake(),
                            slideArm.new wristReady(),
                            new ParallelCommandGroup(
                                    slideArm.new RetractSlideArmPartial(),
                                    this.new IntakeSample1()
                            ),
                            new ParallelCommandGroup(
                                    this.new IntakeHoldSample1().withTimeout(3000),
                                    new SequentialCommandGroup(

                                //new ParallelCommandGroup(
                                        //this.new IntakeSample1(),
                                        slideArm.new PivotSlideArmDown(),
                                        slideArm.new ExtendSlideToSample1(),
                                        new WaitCommand(1000),
                                //slideArm.new wristGather(),

                                        new ParallelCommandGroup(
                                            slideArm.new IntakeSample().withTimeout( AUTO_FIRST_INTAKE_PICKUP_THRESHOLD),

                                                new SequentialCommandGroup(
                                                    new WaitCommand(1000),
                                                    slideArm.new RetractSlideArmPartial()
                                                )
                                        )
                                    )
                            ),
                            slideArm.new stopIntake(),
                            new ScoreSample1(),
                        new ParallelCommandGroup(
                                new ScoreSample1Hold().withTimeout(4000),
                                slideArm.new PivotSlideArmUp(),
                                slideArm.new wristReady(),
                                new SequentialCommandGroup(
                                        slideArm.new ExtendSlideArm(),
                                        slideArm.new wristScore(),
                                        slideArm.new IntakeScore()
                                )
                        )
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
            slideArm.derailShiftDown();
        }
        waitForStart();
        leftLight.setColor(IndicatorLight.COLOR_BEECON);
        rightLight.setColor(IndicatorLight.COLOR_BEECON);
        slideArm.slideBrakes();
        while (opModeIsActive()) {
            follower.update();
            slideArm.update();
            //specimenArm.update();

            // Feedback to Driver Hub
            telemetry.addData("~~path state~~",  pathState);
            telemetry.addData("::pedropathing vals::", "");
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
            scheduler.run();


        }
        OpModeTransfer.autoPose = follower.getPose();

    }
}