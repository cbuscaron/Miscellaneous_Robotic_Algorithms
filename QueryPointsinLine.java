/*
 * Camilo Buscaron
 * 9/2014
 * Algorithm to return the y value for a given x based on a query point[x_q,y_q] on the line
 * Connecting the nearest two points. A Function y= f(x) is defined by a set of {x,y] pairs.
 * 
 * For the given data set of a function to be valid all numbers there must be at least 2 x-y pairs
 * all numbers must be real, and the x points must be presented in increasing order.
 * 
 * If the x points are not in increasing order or there are a few than 2 x-y pairs the program returns
 * no solution
 */


import java.io.*;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Scanner;


public class Solution{
    public static void main(String args[] ) throws Exception {
    	
    
    	Scanner in = new Scanner(System.in);
		int NDataPoints = in.nextInt();
		
		double X_q[] = new double[NDataPoints];
		double Y_q[] = new double[NDataPoints];
    
		if(NDataPoints <2){			
			System.out.print("NoSolution");
			return;
		}
		
		//Receive all the X-Y pairs from the console separated by a single space
		for(int i =0; i<NDataPoints; i++){			
			X_q[i] = in.nextDouble();
			Y_q[i] = in.nextDouble();
		}
		
		// Populate the output and querie array by the input giving the number of query points
		int TestPoints = in.nextInt();
		double X_t[] = new double[TestPoints];
		double Y_o[] = new double[TestPoints];
		
		
		//Receive all the querie points
		for(int i =0; i<TestPoints;i++){X_t[i] = in.nextDouble();}
		
		//checking that all the X points are in increasing order
		for(int i = 0; i<NDataPoints; i++){			
			if(i+1<NDataPoints){	
					if(X_q[i+1]<X_q[i]){
						System.out.print("NoSolution");
						return;				
					}
				}
			}
		
		int c =0; 
		int b =0;
		
		//Determining if the test points are within the connecting line
		//by using an assistance method that calculates the slope and returns the y-intercept
		for(int i=0; i<TestPoints;i++){
				
				if(X_t[i]<X_q[0] || X_t[i] == X_q[0]){
					Y_o[i] = Y_q[0]; 
					c=i;												
				}
				
				else if(X_t[i]>X_q[c] && X_t[i]<X_q[c+1]){	
					Y_o[i]= YofX(X_q[c],X_q[c+1],Y_q[c], Y_q[c+1], X_t[i]);										
					if(c<NDataPoints-2){c =c +1;}
				}
				
				
				else if(X_t[i]>X_q[NDataPoints-1] || X_t[i]==X_q[NDataPoints-1]){Y_o[i] = Y_q[NDataPoints-1];}		
		
				else{i = i-1;c= c+1;}						
		
		}
		
		
		
		//Output Y values for the test points
		for(int i =0; i<TestPoints;i++){			
			System.out.println(round(Y_o[i],1));
		}
		
		
		
		
    
    
    }
    
    
    private static double YofX(double X1, double X2, double Y1, double Y2, double X_t){
	   	
	   double slope = (Y2-Y1)/(X2-X1);	   
	   double Y_q = (slope)*(X_t-X1) + Y1;
	   
	   return Y_q;
	      
   }
   
   public static double round(double value, int places) {
	    if (places < 0) throw new IllegalArgumentException();

	    BigDecimal bd = new BigDecimal(value);
	    bd = bd.setScale(places, RoundingMode.HALF_UP);
	    return bd.doubleValue();
	}
       
    
    
}