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

@Autonomous(name = "SampleAuto")
public class SampleAuto extends LinearOpMode {
    boolean teamChangeRequested = false;
    TeleOpComp.TeamColor HIVE_COLOR = TeleOpComp.TeamColor.TEAM_BLUE;
    public String pathState = null;
    private SlideArm slideArm = null;
    private SpecimenArm specimenArm = null;
    private CommandScheduler scheduler = null;
    private double RAW_DRIVE_TIME = 500.0; //250.0;
    private ElapsedTime intakeLightTimer = new ElapsedTime();
    private ElapsedTime derailRelaxTimer = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private double DRIVE_POWER = 0.3;
    private boolean backupUsed = false;
    private boolean colorSensorUsed = false;
    private boolean derailRelaxed = false;
    private int INTAKE_COLOR_TIMER_FLASH_TIME = 1000;
    private double DERAIL_RELAX_TIME = 750.0;
    public static Pose startPose = new Pose(8,111, Math.toRadians(0));

    private Follower follower;

    public static PathChain preload() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(8, 111, Point.CARTESIAN),
                        new Point(15, 114.3, Point.CARTESIAN),
                        new Point(9.6, 124.0, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
    }
    public static PathChain intake1Sample() {
        return new PathBuilder()
                .addPath(new BezierLine( // after place, goes to first sample
                        new Point(9.6, 124.0, Point.CARTESIAN),
                        new Point(24.25, 121.5, Point.CARTESIAN)))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();
    }
    public static PathChain score1Sample() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(24.25, 121.5, Point.CARTESIAN),
                        new Point(12.9, 127.7, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-40))
                .build();

    }

    public static PathChain intake2Sample() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(12.9, 127.7, Point.CARTESIAN),
                        new Point(23.75, 130.25, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(0))
                .build();
    }

    public static PathChain score2Sample() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(23.75, 130.25, Point.CARTESIAN),
                        new Point(20.1, 127.5, Point.CARTESIAN),
                        new Point(13, 126, Point.CARTESIAN) //17.6, 125.6
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-40))
                .setZeroPowerAccelerationMultiplier(3.0)
                .build();
    }
    public static PathChain intake3Sample() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(13, 126, Point.CARTESIAN), //17.6, 125.6
                        new Point(35.6, 134.5, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(37.5))
                .setZeroPowerAccelerationMultiplier(4.0)//35
                .build();
    }
    public static PathChain score3Sample() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(35.6, 134.5, Point.CARTESIAN),
                        new Point (32,110, Point.CARTESIAN),
                        new Point(10.75, 124.0, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(37.5), Math.toRadians(-45)) //38.5
                .build();
    }
    public static PathChain fixRotation() {
        return new PathBuilder()
                .addPath(new BezierCurve(
                        new Point(10.75, 124.0, Point.CARTESIAN),
                        new Point(47,126, Point.CARTESIAN),
                        new Point(42,107, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .build();
        /*.addPath(new BezierLine(
                new Point(11.7, 131.7, Point.CARTESIAN),
                new Point(12, 129, Point.CARTESIAN)
        ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();*/
    }
    // TODO: intake and place last sample
    // TODO: fix pivot to be smart

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

    public class FollowIntake1Sample extends CommandBase {
        boolean followerStarted = false;
        public FollowIntake1Sample() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(intake1Sample(), 0.8, false);
                pathState = "intake 1st sample";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    public class FollowScore1Sample extends CommandBase {
        boolean followerStarted = false;
        public FollowScore1Sample() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score1Sample(), 0.7, true);
                pathState = "score 2nd sample";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowIntake2Sample extends CommandBase {
        boolean followerStarted = false;
        public FollowIntake2Sample() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(intake2Sample(), 0.8,false);
                pathState = "intake 2nd sample";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    public class FollowScore2Sample extends CommandBase {
        boolean followerStarted = false;
        public FollowScore2Sample() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score2Sample(), 0.8, true);
                pathState = "score 3rd sample";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowIntake3Sample extends CommandBase {
        boolean followerStarted = false;
        public FollowIntake3Sample() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(intake3Sample(), 0.7, true);
                pathState = "intake 4th sample";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class FollowScore3Sample extends CommandBase {
        boolean followerStarted = false;
        public FollowScore3Sample() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(score3Sample(), 0.8,true);
                pathState = "score 4th sample,  royd doubted";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

    public class FollowFixRotation extends CommandBase {
        boolean followerStarted = false;
        public FollowFixRotation() {}
        @Override
        public void initialize() {
            followerStarted = false;
        }
        @Override
        public void execute() {
            if (!followerStarted) {
                follower.followPath(fixRotation(), 1, true);
                pathState = "fix rotation";
                followerStarted = true;
            }
        }
        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class IntakeSample extends CommandBase {
        boolean isDone = false;
        public IntakeSample() {}

        @Override
        public void initialize() {
            isDone = false;
        }

        @Override
        public void execute() {
            isDone = slideArm.activateIntakeWithSensorSampleAuto();
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }
        @Override
        public void end(boolean interrupted) {
            if(interrupted) {
                backupUsed = true;
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


        IndicatorLight leftLight = new IndicatorLight(hardwareMap, telemetry, "left_light");
        IndicatorLight rightLight = new IndicatorLight(hardwareMap, telemetry, "right_light");
        leftLight.setColor(IndicatorLight.COLOR_RED);
        rightLight.setColor(IndicatorLight.COLOR_RED);

        derailRelaxed = false;
        backupUsed = false;
        colorSensorUsed = false;
        derailRelaxed = false;


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
                        slideArm.new RetractSlideArm().withTimeout(1500),
                        new ParallelCommandGroup(
                                slideArm.new PivotSlideArmDown().withTimeout(1000),
                                this.new FollowIntake1Sample()
                        ),
                        slideArm.new wristGather(),
                        this.new IntakeSample().withTimeout((long) SlideArm.AUTO_FIRST_INTAKE_PICKUP_THRESHOLD),
                        slideArm.new stopIntake(),
                        new ParallelCommandGroup(
                                this.new FollowScore1Sample(),
                                slideArm.new wristReady(),
                                slideArm.new PivotSlideArmUp()
                        ),
                        slideArm.new ExtendSlideArm(),
                        slideArm.new wristScore(),
                        new WaitCommand(250),
                        slideArm.new IntakeScore(),

                        slideArm.new stopIntake(),
                        slideArm.new wristReady(),
                        slideArm.new RetractSlideArm().withTimeout(1500),
                        new ParallelCommandGroup(
                                slideArm.new PivotSlideArmDown().withTimeout(1000),
                                this.new FollowIntake2Sample()
                        ),
                        slideArm.new wristGather(),
                        this.new IntakeSample().withTimeout((long) SlideArm.AUTO_INTAKE_PICKUP_THRESHOLD),

                        slideArm.new stopIntake(),
                        new ParallelCommandGroup(
                                this.new FollowScore2Sample(),
                                slideArm.new wristReady(),
                                slideArm.new PivotSlideArmUp()
                        ),
                        slideArm.new ExtendSlideArm(),
                        slideArm.new wristScore(),
                        new WaitCommand(250),
                        slideArm.new IntakeScore(),
                        slideArm.new stopIntake(),
                        slideArm.new wristReady(),
                        slideArm.new RetractSlideArmTo3Sample().withTimeout(1500),
                        new ParallelCommandGroup(
                                slideArm.new PivotSlideArmDown().withTimeout(1000),
                                this.new FollowIntake3Sample()
                                //slideArm.new ExtendSlideToSample1()
                        ),
                        new WaitCommand(500),
                        slideArm.new wristGather(),
                        this.new IntakeSample().withTimeout((long) SlideArm.AUTO_INTAKE_DIAG_PICKUP_THRESHOLD),
                        slideArm.new stopIntake(),
                        new ParallelCommandGroup(
                                this.new FollowScore3Sample(),
                                slideArm.new wristReady(),
                                slideArm.new PivotSlideArmUp()
                        ),
                        slideArm.new ExtendSlideArm(),
                        slideArm.new wristScore(),
                        new WaitCommand(250),
                        slideArm.new IntakeScore(),
                        //new WaitCommand(500),
                        slideArm.new stopIntake(),
                        slideArm.new wristReady(),
                        slideArm.new RetractSlideArm().withTimeout(1500),
                        new ParallelCommandGroup(
                                slideArm.new PivotSlideArmDown().withTimeout(1000),
                                specimenArm.new GoToAutoBar(),
                                slideArm.new wristReady(),
                                this.new FollowFixRotation()
                        ),
                        this.new RawDriveForward(),
                        specimenArm.new ArmToLimp(),
                        new WaitCommand(2000)
                        //this.new FollowFixRotation()
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
        slideArm.derailShiftDown(); // THIS IS THE WRONG WAY
        //slideArm.derailRelax();
        //derailRelaxTimer.reset();
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
            slideArm.addSlideTelemetry();
            telemetry.update();
            scheduler.run();

            if (colorSensorUsed) {
                if (INTAKE_COLOR_TIMER_FLASH_TIME < intakeLightTimer.milliseconds()) {
                    colorSensorUsed = false;
                }
                rightLight.setColor(IndicatorLight.COLOR_GREEN);
                leftLight.setColor(IndicatorLight.COLOR_BEECON);
            }
            else if (backupUsed) {
                if (INTAKE_COLOR_TIMER_FLASH_TIME < intakeLightTimer.milliseconds()) {
                    backupUsed = false;
                }
                rightLight.setColor(IndicatorLight.COLOR_RED);
                leftLight.setColor(IndicatorLight.COLOR_BEECON);
            }
            else {
                leftLight.setColor(IndicatorLight.COLOR_BEECON);
                rightLight.setColor(IndicatorLight.COLOR_BEECON);
            }
            /*
            if(!derailRelaxed && derailRelaxTimer.milliseconds() >= DERAIL_RELAX_TIME){
                slideArm.derailRelax();
                derailRelaxed = true;
            }
            */


        }
        OpModeTransfer.autoPose = follower.getPose();

    }
}