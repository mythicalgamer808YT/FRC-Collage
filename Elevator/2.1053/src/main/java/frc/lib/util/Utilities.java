package frc.lib.util;

public class Utilities {
    public static final double sparkFlexResolution = 7168;

    public static double polynomialAccleration(double x) {
        return Math.pow(x,3) * 0.795903 + x * 0.203938;
    }

    //double[relInnerHeight, relPrimaryHeight]
    public static double[] distributeHeights(double curInner, double curPrimary, double newTotalHeight) {
        double newInner = (newTotalHeight + curInner - curPrimary) / 2.0;
        double newPrimary = newTotalHeight - curInner;

        if(newInner < 0) {
            newInner = 0;
            newPrimary = newTotalHeight;
        } else if(newInner > 1) {
            newInner = 1;
            newPrimary = newTotalHeight - 1;
        }

        if(newPrimary < 0) {
            newPrimary = 0;
            newInner = newTotalHeight;
        } else if(newPrimary > 1) {
            newPrimary = 1;
            newInner = newTotalHeight - 1;
        }

        return new double[] {Math.max(0, Math.min(1, newInner)), Math.max(0, Math.min(1, newPrimary))};
    }
}