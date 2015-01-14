/*
 * Cam. Buscaron
 * 9/2014
 * 
 *This program returns a string of 'true' or 'false' depending whether a set of test points lie strictly 
 *inside a given rectangle of with variable orientation 'yaw'
 * 
 * 
 * 
 */




import java.awt.List;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.Scanner;


public class Solution{

	public static void main(String[] args) {
		

		/*Inputs Xd with of the rectangle
		         Yd Height of the rectangle
		       	 X postion center of the rectangle
		         Y position center of the rectangle
		         yaw in radians  
		
		*/
		
		/* Strategy */
		// 1) Find the four corners of the rectangle
		// 2) Transform the rectangle by the yaw, find the new 4 Corners
		// 3) Find the min and max of each axis
		// 4) check if the query points are within the bounded new four corners
		
		
		double Xd = 0;
		
		double Yd = 0;
		
		double Xp = 0;
		double Yp = 0;
		double yaw = 0;
		
		Scanner in = new Scanner(System.in);
		
		ArrayList<Double> Xq = new ArrayList<Double>();
		ArrayList<Double> Yq = new ArrayList<Double>();
		
		while(in.hasNext()){
			
			String line = in.nextLine();
			Scanner lineScan = new Scanner(line);
			
			
			lineScan.useDelimiter("\\s*, \\s*");
			

		
			if(line.length()>20){
			Xd = lineScan.nextDouble();			
			Yd = lineScan.nextDouble();
			Xp = lineScan.nextDouble();
			Yp = lineScan.nextDouble();
			yaw = lineScan.nextDouble();
			}
			
			else if(line.length()>4 || line.length()==4 ){				
				Xq.add(lineScan.nextDouble());
				Yq.add(lineScan.nextDouble());
			}
			
			
			else{break;}
			
			
		}
		
		
		
				
				
			/*	System.out.println("-----------Variables-----------");
				System.out.println("Xd:"+Xd);
				System.out.println("Yd:"+Yd);
				System.out.println("Xp:"+Xp);
				System.out.println("Yp:"+Yp);
				System.out.println("Yaw:"+yaw);
		
		
			/*	System.out.println("-----------X--Values on the list-----------");*//*
				
				for(int i= 0; i<Xq.size(); i++){
					System.out.println("X"+i+":"+Xq.get(i));
				}
			
				System.out.println("-----------Y--Values on the list-----------");
		
				for(int i= 0; i<Yq.size(); i++){			
				System.out.println("X"+i+":"+Yq.get(i));
				}*/
			
			
			//System.out.println("-----------CornerPoints of the Square-----------");
			double Xmin = round(Xmin(Xd,Yd,Xp,Yp,yaw),2);
			double Xmax = round(Xmax(Xd,Yd,Xp,Yp,yaw),2);
			
			double Ymin = round(Ymin(Xd,Yd,Xp,Yp,yaw),2);
			double Ymax = round(Ymax(Xd,Yd,Xp,Yp,yaw),2);
			
			//System.out.println("Corner1 " +"("+Xmin+","+Ymax+")");
			//System.out.println("Corner2 " +"("+Xmax+","+Ymax+")");
			//System.out.println("Corner3 " +"("+Xmax+","+Ymin+")");
			//System.out.println("Corner4 " +"("+Xmin+","+Ymin+")");
			
			//System.out.println("-----------True/False Output-----------");
			
			for(int i =0; i<Xq.size(); i++){
				
				if(Xq.get(i)> Xmin && Xq.get(i)< Xmax && Yq.get(i)> Ymin && Yq.get(i)< Ymax){System.out.println("true");}
				
				else{System.out.println("false");}
				
			}
			
	}
	
		public static double Xmin(double Xd, double Yd, double Xp, double Yp, double yaw){
			//Find the Corner points
			
			double X1= -Xd/2 + Xp;
			double X2=  Xd/2 + Xp;
			double X3=  Xd/2 + Xp;
			double X4= -Xd/2 + Xp;
			
			double Y1=  Yd/2 + Yp;
			double Y2=  Yd/2 + Yp;
			double Y3= -Yd/2 + Yp;
			double Y4= -Yd/2 + Yp;
			
			double X11= X1*Math.cos(yaw) + Y1*Math.sin(yaw);
			double X21= X2*Math.cos(yaw) + Y2*Math.sin(yaw);
			double X31= X3*Math.cos(yaw) + Y3*Math.sin(yaw);
			double X41= X4*Math.cos(yaw) + Y4*Math.sin(yaw);
			
			double Xmin = Math.min(Math.min(X11,X21),Math.min(X31,X41));
									
				return Xmin;
		
		}

		public static double Xmax(double Xd, double Yd, double Xp, double Yp, double yaw){
	
			double X1= -Xd/2 + Xp;
			double X2=  Xd/2 + Xp;
			double X3=  Xd/2 + Xp;
			double X4= -Xd/2 + Xp;
			
			double Y1=  Yd/2 + Yp;
			double Y2=  Yd/2 + Yp;
			double Y3= -Yd/2 + Yp;
			double Y4= -Yd/2 + Yp;
			
			double X11= X1*Math.cos(yaw) + Y1*Math.sin(yaw);
			double X21= X2*Math.cos(yaw) + Y2*Math.sin(yaw);
			double X31= X3*Math.cos(yaw) + Y3*Math.sin(yaw);
			double X41= X4*Math.cos(yaw) + Y4*Math.sin(yaw);
						
			double Xmax = Math.max(Math.max(X11,X21),Math.max(X31,X41));
									
				return Xmax;
		
	
		}

		public static double Ymin(double Xd, double Yd, double Xp, double Yp, double yaw){
			double X1= -Xd/2 + Xp;
			double X2=  Xd/2 + Xp;
			double X3=  Xd/2 + Xp;
			double X4= -Xd/2 + Xp;
			
			double Y1=  Yd/2 + Yp;
			double Y2=  Yd/2 + Yp;
			double Y3= -Yd/2 + Yp;
			double Y4= -Yd/2 + Yp;
			
			double Y11= -X1*Math.sin(yaw)+ Y1*Math.cos(yaw);
			double Y21= -X2*Math.sin(yaw)+ Y2*Math.cos(yaw);
			double Y31= -X3*Math.sin(yaw)+ Y3*Math.cos(yaw);
			double Y41= -X4*Math.sin(yaw)+ Y4*Math.cos(yaw);
			
			double Ymin = Math.min(Math.min(Y11,Y21),Math.min(Y31,Y41));
			
				return Ymin;
	
		}

		public static double Ymax(double Xd, double Yd, double Xp, double Yp, double yaw){
			double X1= -Xd/2 + Xp;
			double X2=  Xd/2 + Xp;
			double X3=  Xd/2 + Xp;
			double X4= -Xd/2 + Xp;
			
			double Y1=  Yd/2 + Yp;
			double Y2=  Yd/2 + Yp;
			double Y3= -Yd/2 + Yp;
			double Y4= -Yd/2 + Yp;
			
			double Y11= -X1*Math.sin(yaw)+ Y1*Math.cos(yaw);
			double Y21= -X2*Math.sin(yaw)+ Y2*Math.cos(yaw);
			double Y31= -X3*Math.sin(yaw)+ Y3*Math.cos(yaw);
			double Y41= -X4*Math.sin(yaw)+ Y4*Math.cos(yaw);
			
			double Ymax = Math.max(Math.max(Y11,Y21),Math.max(Y31,Y41));
			
				return Ymax;
	
			}

		
		public static double round(double value, int places) {
		    if (places < 0) throw new IllegalArgumentException();

		    BigDecimal bd = new BigDecimal(value);
		    bd = bd.setScale(places, RoundingMode.HALF_UP);
		    return bd.doubleValue();
		}
}
