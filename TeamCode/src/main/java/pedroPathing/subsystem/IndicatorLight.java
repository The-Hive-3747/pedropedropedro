package pedroPathing.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IndicatorLight {
    Servo light = null;
    public static double COLOR_RED = 0.279;
    public static double COLOR_ORANGE = 0.333;
    public static double COLOR_YELLOW = 0.388;
    public static double COLOR_SAGE = 0.444;
    public static double COLOR_GREEN = 0.5;
    public static double COLOR_AZURE = 0.555;
    public static double COLOR_BLUE = 0.611;
    public static double COLOR_INDIGO = 0.666;
    public static double COLOR_VIOLET = 0.722;
    public static double COLOR_WHITE = 1.0;
    public static double COLOR_OFF = 0.0;
    public static double COLOR_BEECON = 0.353;
    HardwareMap hardwareMap = null;
    Telemetry telemetry = null;
    String name = null;

    public IndicatorLight(HardwareMap hm, Telemetry tele, String mapName){
        hardwareMap=hm;
        telemetry=tele;
        name=mapName;
        light = hardwareMap.get(Servo.class,name);
    }

    public void setColor(double color){
        light.setPosition(color);
    }

}
