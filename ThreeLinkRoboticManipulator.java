/*
 * Camilo Buscaron
 * 9/2014
 * 
 * This algorithm figures out the x-y position of the end-effector on a 3 Links Robotic Manipulator
 * 
 * The Base of the first link is connected to the ground through a rotational joint
 * The angle of this joint is given by t1
 * The angle of the second link relative to the first link is given by t2
 * Along the second link is a prismatic joint
 * The length of the second link is given by x, where x is always positive
 * 
 * I solved this problem robustly with a little trigonometry
 * This best way to analyze this is to actually draw it in paper
 */


import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Scanner;


public class Solution{

	public static void main(String[] args) {
		Scanner in = new Scanner(System.in);
		double t1 = in.nextDouble();
		double t2 = in.nextDouble();
		double x= in.nextDouble();
		
		double L1 = 2;
		
		double Xp = Xposition(L1,x,t1,t2);
		double Yp = Yposition(L1,x,t1,t2);
		
				
		BigDecimal t= new BigDecimal(Xp);
		BigDecimal tp= new BigDecimal(Yp);
		
		System.out.print((round(t,2)));
		System.out.print(" ");
		System.out.print((round(tp,2)));
	
		
	}

	
	public static double Xposition(double L1, double X, double t1, double t2){
		double Xp =L1*Math.cos(t1)+ X*Math.cos(t1+t2);
		return Xp;
		
	}
	
	public static double Yposition(double L1, double X, double t1, double t2){		
		double Yp = L1*Math.sin(t1)+ X*Math.sin(t1+t2);	
		return Yp;		
		
	}
	
	public static BigDecimal round(BigDecimal value, int places) {
	    if (places < 0) throw new IllegalArgumentException();

	    BigDecimal bd = value.setScale(places, RoundingMode.HALF_UP);
	    return bd;
	}
}
