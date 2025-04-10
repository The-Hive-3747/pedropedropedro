package pedroPathing.examples;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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
import com.pedropathing.pathgen.Path;
import com.pedropathing.util.Constants;
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
@Autonomous(name = "Five Specimen Tester")
public class FiveSpecimenAutoTester extends LinearOpMode {
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
                        new Point(8.5, 61.5, Point.CARTESIAN),
                        new Point(37, 75, Point.CARTESIAN))) //75
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public static PathChain driveForward() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(8, 61.5, Point.CARTESIAN),
                        new Point(8.5, 61.5, Point.CARTESIAN)
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

    public class FollowDriveForward extends CommandBase {
        boolean followerStarted = false;
        public FollowDriveForward() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(driveForward());
                pathState = "drive straight";
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
    @Override
    public void runOpMode() {
        //Constants.setConstants(FConstants.class, LConstants.class);
        //Constants.setConstants(FConstants.class,LConstants.class);
        follower = new Follower(hardwareMap,FConstants.class,LConstants.class);
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

        //buildPaths();
        scheduler = CommandScheduler.getInstance();

        scheduler.schedule(
                new SequentialCommandGroup(
                        specimenArm.new SpecimenArmEnter(),
                        new WaitCommand(2000),
                        this.new FollowDriveForward(),
                        this.new FollowPreload()
                        /*
                        new ParallelCommandGroup(
                                slideArm.new wristStow(),
                                specimenArm.new doAutoClawStateClose(),
                                specimenArm.new SpecimenArmEnter(),
                                this.new FollowDriveForward() // SCORE PRELOAD
                        ),

                         */
                        //this.new RawDriveForward(),
                        //specimenArm.new SpecimenArmScore(),
                        //specimenArm.new doAutoClawStateOpen(),
                        //specimenArm.new SpecimenArmCollect()
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

        //slideArm.slideBrakes();
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
        OpModeTransfer.autoColor = HIVE_COLOR;
    }
}