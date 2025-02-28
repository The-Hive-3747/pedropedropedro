package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import pedroPathing.subsystem.IndicatorLight;
import pedroPathing.subsystem.SpecimenArm;

@TeleOp(name="MotorTickReaderOpMode")
public class MotorTickReaderOpMode extends LinearOpMode {
    public static enum SampleColor {SAMPLE_BLUE, SAMPLE_RED, SAMPLE_YELLOW}
    private DcMotor pivotMotor = null;
    private DcMotor leftSlideMotor = null;
    private DcMotor rightSlideMotor = null;
    private DcMotor rightShoulder = null;
    //private HardwareMap hardwareMap = null;
    //private Telemetry telemetry = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    //private RevColorSensorV3 intakeColor = null;
    private Servo wrist = null;
    private ElapsedTime colortimer = new ElapsedTime();
    private double COLOR_TIME_THRESHOLD = 150.0;
    private double COLOR_MAX = 0.722;
    private double COLOR_CHANGE = 0.001;
    private double COLOR_START = 0.279;
    private double color = COLOR_START;
    public static double LUMINANCE_BLUE_CB_MIN = -16.0;
    public static double LUMINANCE_RED_CR_MIN = -34.0;
    private static double LUMINANCE_RED = 0.299;
    private static double LUMINANCE_GREEN = 0.587;
    private static double LUMINANCE_BLUE = 0.114;
    private static double LUMINANCE_PB = 0.564;
    public static double LUMINANCE_PR = 0.713;
    private boolean increasing_color = true;
    @Override
    public void runOpMode() throws InterruptedException {

        IndicatorLight leftLight = new IndicatorLight(hardwareMap, telemetry, "left_light");
        IndicatorLight rightLight = new IndicatorLight(hardwareMap, telemetry, "right_light");

        leftLight.setColor(IndicatorLight.COLOR_RED);
        rightLight.setColor(IndicatorLight.COLOR_RED);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivotMotor = hardwareMap.get(DcMotor.class,"pivot_motor");
        leftSlideMotor = hardwareMap.get(DcMotor.class,"left_slide_motor" );
        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_slide_motor");
        rightShoulder = hardwareMap.get(DcMotor.class, "right_shoulder");
        leftIntake = hardwareMap.get(CRServo.class, "intake_left");
        rightIntake = hardwareMap.get(CRServo.class, "intake_right");
        //intakeColor = hardwareMap.get(RevColorSensorV3.class, "intake_color");
        wrist = hardwareMap.get(Servo.class, "wrist");

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //TODO: set polarity of motors
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShoulder.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Slide arm status", "init complete");
        leftLight.setColor(IndicatorLight.COLOR_GREEN);
        rightLight.setColor(IndicatorLight.COLOR_GREEN);
        waitForStart();


        leftLight.setColor(IndicatorLight.COLOR_BEECON);
        rightLight.setColor(IndicatorLight.COLOR_BEECON);

        while (opModeIsActive()) {

            if (colortimer.milliseconds() > COLOR_TIME_THRESHOLD) {
                if (increasing_color) {
                    color += COLOR_CHANGE;
                    if (color > COLOR_MAX) {
                        color = COLOR_MAX;
                        increasing_color = false;  // Start fading back
                    }
                } else {
                    color -= COLOR_CHANGE;
                    if (color < COLOR_START) {
                        color = COLOR_START;
                        increasing_color = true;  // Start increasing again
                    }
                }
            }

            leftLight.setColor(color);
            rightLight.setColor(color);
            telemetry.addData("Left Slide Ticks", leftSlideMotor.getCurrentPosition() );
            telemetry.addData("Right Slide Ticks", rightSlideMotor.getCurrentPosition());
            telemetry.addData("Pivot Motor Ticks", pivotMotor.getCurrentPosition());
            telemetry.addData("Right Shoulder Ticks", rightShoulder.getCurrentPosition());
            telemetry.addData("Wrist Ticks", wrist.getPosition());
            //telemetry.addData("Color Distance (cm)", intakeColor.getDistance(DistanceUnit.CM));
            //telemetry.addData("Team Color", detectColor());
            //telemetry.addData("Red", intakeColor.red());
            //telemetry.addData("Blue", intakeColor.blue());
            //telemetry.addData("Green", intakeColor.green());
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            //Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
    /*
    public double getLuminancePb() {
        return LUMINANCE_PB*(intakeColor.blue())-(intakeColor.red()*LUMINANCE_RED+
                LUMINANCE_GREEN*intakeColor.green()+LUMINANCE_BLUE*intakeColor.blue());
    }
    public double getLuminancePr() {
        return LUMINANCE_PR*(intakeColor.red())-(intakeColor.red()*LUMINANCE_RED+
                LUMINANCE_GREEN*intakeColor.green()+LUMINANCE_BLUE*intakeColor.blue());
    }
    public SampleColor detectColor(){
        if (LUMINANCE_BLUE_CB_MIN < getLuminancePb()){
            return SampleColor.SAMPLE_BLUE;
        }
        else if (LUMINANCE_RED_CR_MIN < getLuminancePr()) {
            return SampleColor.SAMPLE_RED;
        }
        return SampleColor.SAMPLE_YELLOW;
    }*/
}
