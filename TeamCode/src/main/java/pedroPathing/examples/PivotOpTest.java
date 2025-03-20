package pedroPathing.examples;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import pedroPathing.TeleOpComp;
import pedroPathing.subsystem.IndicatorLight;
import pedroPathing.subsystem.OpModeTransfer;
import pedroPathing.subsystem.SlideArm;
import pedroPathing.subsystem.SpecimenArm;

//@Disabled
@TeleOp(name="pivot test file")
public class PivotOpTest extends LinearOpMode {
    //private SpecimenArm specimenArm = null;
    //private CommandScheduler scheduler = null;
    //private SlideArm slideArm = null;
    private DcMotor pivotMotor = null;
    private boolean upRequested = false;
    private boolean downRequested = false;
    @Override
    public void runOpMode() {
        //specimenArm = new SpecimenArm(hardwareMap, telemetry, true);
        //slideArm = new SlideArm(hardwareMap, telemetry, true);
        IndicatorLight leftLight = new IndicatorLight(hardwareMap, telemetry, "left_light");
        IndicatorLight rightLight = new IndicatorLight(hardwareMap, telemetry, "right_light");
        pivotMotor = hardwareMap.get(DcMotor.class,"pivot_motor");

        waitForStart();


        while (opModeIsActive()) {
            //scheduler.run();
            if (gamepad1.dpad_up && !upRequested){
                upRequested = true;
                pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                pivotMotor.setPower(0.8);
                pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (!gamepad1.dpad_up && upRequested){
                upRequested = false;
                pivotMotor.setPower(0.0);
                pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (gamepad1.dpad_down && !downRequested){
                downRequested = true;
                pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                pivotMotor.setPower(-0.8);
                pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (!gamepad1.dpad_down && downRequested){
                downRequested = false;
                pivotMotor.setPower(0.0);
                pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }


    }
}
