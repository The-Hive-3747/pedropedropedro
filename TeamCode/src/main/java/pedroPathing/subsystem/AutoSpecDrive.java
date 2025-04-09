package pedroPathing.subsystem;

import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.fasterxml.jackson.core.TreeNode;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoSpecDrive {
    CommandScheduler scheduler = CommandScheduler.getInstance();

    Follower follower = null;
    public static Pose intakePose = null;
    SpecimenArm specimenArm = null;
    private double RAW_DRIVE_TIME = 200.0; //250.0;
    public static DcMotor leftFront = null;
    public static DcMotor rightFront = null;
    public static DcMotor leftBack = null;
    public static DcMotor rightBack = null;
    public static boolean autoSpecCycleIsDone = true;
    private double DRIVE_POWER = 0.3;
    private String pathState = "";
    private Pose intPose = null;
    public static Pose iPose = new Pose(0,0,0);
    private PathChain intakeSpecimenPath = null;
    private PathChain scoreSpecimenPath = null;


    public AutoSpecDrive(Follower f, SpecimenArm SpeciArm) {
        follower = f;
        specimenArm = SpeciArm;
        intakePose = new Pose(0,0,0);
        intakeSpecimenPath = intakeSpecimen();
        scoreSpecimenPath = scoreSpecimen();
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


        public PathChain scoreSpecimen() {
            return new PathBuilder()
                    .addPath(new BezierCurve(
                            new Point(14,42, Point.CARTESIAN),
                            new Point(15, 70, Point.CARTESIAN),
                            new Point(39, 69.75, Point.CARTESIAN)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

    }
    public PathChain slideSpecimen() {
        return new PathBuilder()
                .addPath(new BezierLine(
                        new Point(40,69.75, Point.CARTESIAN),
                        new Point(40, 71.3, Point.CARTESIAN) //x:38
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

    }


        public PathChain intakeSpecimen() {
            return new PathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(40, 71.3, 0), //x:38
                            new Pose(31, 73, 0),
                            new Pose(20, 45, 0)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(new BezierLine(
                            new Pose(20, 45, 0),
                            new Pose(14,42,0)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

    }


    public class autonomousSpecPlace extends CommandBase {
        boolean isDone = false;


        public autonomousSpecPlace() {}

        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            if (!isDone) {
                follower.followPath(scoreSpecimenPath);
                isDone = true;
            }
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class autonomousSlideMover extends CommandBase {
        boolean isDone = false;


        public autonomousSlideMover() {}

        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            if (!isDone) {
                follower.followPath(slideSpecimen());
                isDone = true;
            }
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public class autonomousSpecIntake extends CommandBase {
        boolean isDone = false;
        public autonomousSpecIntake() {
        }

        @Override
        public void initialize() {
            isDone = false;
        }
        @Override
        public void execute() {
            if (!isDone) {
                follower.followPath(intakeSpecimenPath);
                isDone = true;
            }
            if (!follower.isBusy()) {
                autoSpecCycleIsDone = true;
            }
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }
    public boolean isAutoSpecCycleIsDone() {
        return autoSpecCycleIsDone;
    }
    public void setIntakePose() {
        intakePose = follower.getPose().copy();
    }
    public void resetScheduler() {
        autoSpecCycleIsDone = false;
        scheduler.schedule(
                new SequentialCommandGroup(
                        specimenArm.new doAutoClawStateClose(),
                        specimenArm.new SpecimenArmEnter(),
                        this.new autonomousSpecPlace(),
                        this.new RawDriveForward(),
                        specimenArm.new SpecimenArmScore(),
                        this.new autonomousSlideMover(),
                        specimenArm.new doAutoClawStateOpen(),
                          specimenArm.new SpecimenArmCollect(),
                        this.new autonomousSpecIntake()
                )
        );
    }
    public void runSchedule() {
        scheduler.run();
    }

}
