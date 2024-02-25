package org.firstinspires.ftc.teamcode.function;
import java.util.Iterator;
import java.util.ArrayList;

public class Average {
    public static double arrayavg(double[] array){
        double sum = 0;

        for (int i = 0; i < array.length; i++){
            sum = sum+array[i];
        }
        return sum/array.length;
    }

}
