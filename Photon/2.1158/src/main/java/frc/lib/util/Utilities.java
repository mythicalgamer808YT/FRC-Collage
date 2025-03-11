package frc.lib.util;

public class Utilities {
    public static final double sparkFlexResolution = 7168;

    public static double polynomialAccleration(double x) {
        return Math.pow(x,3) * 0.795903 + x * 0.203938;
    }

    //double[relInnerHeight, relPrimaryHeight]
    public static double[] distributeElevatorHeights(double relTotalHeight, double relCurInner, double relCurPrimary) {
        double newRelInnerHeight = relCurInner + relTotalHeight / 2;
        double newRelPrimaryHeight = relCurPrimary + relTotalHeight / 2;

        if(newRelInnerHeight > 1) {
            double overflow = newRelInnerHeight - 1;
            newRelInnerHeight = 1;
            newRelPrimaryHeight = Math.min(newRelPrimaryHeight + overflow, 1);
        } else if(newRelInnerHeight < 0) {
            double deficit = newRelInnerHeight;
            newRelInnerHeight = 0;
            newRelPrimaryHeight = Math.max(newRelPrimaryHeight - deficit, 0);
        }

        if(newRelPrimaryHeight > 1) {
            double overflow = newRelPrimaryHeight - 1;
            newRelPrimaryHeight = 1;
            newRelInnerHeight = Math.min(newRelInnerHeight + overflow, 1);
        } else if(newRelPrimaryHeight < 0) {
            double deficit = newRelPrimaryHeight;
            newRelPrimaryHeight = 0;
            newRelInnerHeight = Math.max(newRelInnerHeight - deficit, 0);
        }

        return new double[]{newRelInnerHeight, newRelPrimaryHeight};

    }

            //ik its not dynamic but idc, get outta here
    public static double calculateTotalRelHeight(double primaryHeight, double innerHeight) {
        double relPrimaryHeight = (primaryHeight - 1) / (107);//108 - 1
        double relInnerHeight = (innerHeight - 1.2) / (22.5 - 1.2);

        return relPrimaryHeight + relInnerHeight;
    }
}