package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sensor {
        private ColorSensor colorSensorOne;
        private ColorSensor colorSensorTwo;

        public Sensor(){
        }

        public boolean isTape(){
            if((colorSensorOne.blue() >= 150) || (colorSensorOne.red() >= 150)) {
                return true;
            }

            return false;
        }

        public boolean isCapStone(){
            if((colorSensorTwo.red() <= 100)){
                return true;
            }
            return false;
        }

    }

