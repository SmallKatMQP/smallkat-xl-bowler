package com.neuronrobotics.smallkat;

import com.neuronrobotics.bowlerstudio.BowlerStudio;
import com.neuronrobotics.sdk.addons.kinematics.DHChain;
import com.neuronrobotics.sdk.addons.kinematics.DhInverseSolver;
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import java.util.ArrayList;

import com.neuronrobotics.sdk.addons.kinematics.DHChain;
import com.neuronrobotics.sdk.addons.kinematics.DHLink;
import com.neuronrobotics.sdk.addons.kinematics.DhInverseSolver;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import com.neuronrobotics.sdk.common.Log;
import Jama.Matrix;
import eu.mihosoft.vrl.v3d.Transform;

public class scriptJavaIKModel implements DhInverseSolver {

	int limbIndex =0;
	public scriptJavaIKModel(int index){
		limbIndex=index;
	}

	@Override
	public double[] inverseKinematics(TransformNR target, double[] jointSpaceVector, DHChain chain) {
		System.out.println("My IK");
        try {
            ArrayList<DHLink> links = chain.getLinks();
            // THis is the jacobian for the given configuration
            //Matrix jacobian =  chain.getJacobian(jointSpaceVector);
            Matrix taskSpacMatrix = target.getMatrixTransform();

            int linkNum = jointSpaceVector.length;

            double z = target.getZ();
            double y = target.getY();
            double x = target.getX();
            RotationNR q = target.getRotation();
             
            System.out.println("z: " + z);
            System.out.println("y: " + y);
            System.out.println("x: " + x);
            
            double Oang = Math.PI/2 + q.getRotationElevation();
            double Oanginv = (Math.PI/2) - Oang;

            double l1_d = links.get(0).getR();
            double l2_d = links.get(1).getR();
            double l3_d = links.get(2).getR();
            double l4_d = links.get(3).getR();

            System.out.println("L1: " + l1_d);
            System.out.println("L2: " + l2_d);
            System.out.println("L3: " + l3_d);
            System.out.println("L4: " + l4_d);
            
            
            double[] inv = new double[linkNum];

			double a1 = Math.atan2(y , x);
			double a1d = Math.toDegrees(a1);
			double a2 = Math.atan2(z,x); // Z angle using x axis and z axis
			double a2d = Math.toDegrees(a2);
			double theta_1 = a1;
	        double theta1d = Math.toDegrees(theta_1);	            
            System.out.println("The first link angle = "+theta1d);

            double r1 = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // X and Y plane Vector
            double r2 = Math.sqrt(Math.pow(x, 2) + Math.pow(y,2)+Math.pow(z, 2)); // Leg Vector
            double r3 = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2)); // x and z vector
            
            double a3 = Math.atan2(z,r1); // angle of leg vector in plane using xyz
            double a3d = Math.toDegrees(a3);
           
            double x1 = r1; // Rotated frame to line up with leg vector
            double y1 = z; // y value remains the same
            double x2 = Math.sin(Oang)*l4_d;
            double y2 = Math.cos(Oang)*l4_d;

            double Px = x1 - x2;
            double Py = y1 - y2;
            double Wx = x1 - x2; // In leg vector frame
    		double Wy = Math.sin(a1)* Wx; // Wrist Center z in Global Plane
            	   Wx = Math.cos(a1)* Wx; // Wrist Center x in Global Plane
    		double Wz = y1 - y2; // Wrist Center y in Global Plane
    		
    		double theta3_1 = Math.acos(((Math.pow(Px, 2) + (Math.pow(Py, 2))) - (Math.pow(l2_d, 2) + (Math.pow(l3_d, 2)))) / (2*l2_d*l3_d));			
    		double theta3_2 = Math.acos(((Math.pow(l2_d, 2) + (Math.pow(l3_d, 2))) - (Math.pow(Px, 2) + (Math.pow(Py, 2)))) / (2*l2_d*l3_d));
    		
    		double B = Math.atan2(Py , Px);
    		double Y = Math.acos((Math.pow(Px, 2) + Math.pow(Py, 2) + Math.pow(l2_d, 2) - Math.pow(l3_d, 2)) / (2 * l2_d * Math.sqrt(Math.pow(Px, 2) + Math.pow(Py, 2))));
    		double theta2_1 = B - Y;
            double theta2_2 = B + Y;
    		
            double ang1 = (Math.PI-(-theta2_1+theta3_2));
            double theta4_1 = (((Math.PI - ang1) + Oanginv)) - Math.PI;
            double ang2 = (theta2_2+theta3_2);
            double theta4_2 = ((Math.PI-ang2)+Oanginv);
            
            
            System.out.println(theta_1);
            System.out.println(theta2_1);
            System.out.println(theta3_1);
            System.out.println(theta4_1);


		if(limbIndex == 0) {
             //Link 1
             inv[0] = Math.toDegrees(theta_1);
             inv[1] = Math.toDegrees(theta2_1);
             inv[2] = Math.toDegrees(theta3_1);
             inv[3] = Math.toDegrees(theta4_1);
             System.out.println(inv[0]);
             System.out.println(inv[1]);
             System.out.println(inv[2]);
             System.out.println(inv[3]);
            }
            
           if(limbIndex == 1) {
            //Link 2
            inv[0] = Math.toDegrees(theta_1);
            inv[1] = Math.toDegrees(-theta2_1);
            inv[2] = Math.toDegrees(-theta3_1);
            inv[3] = Math.toDegrees(-theta4_1);
            System.out.println(inv[0]);
            System.out.println(inv[1]);
            System.out.println(inv[2]);
            System.out.println(inv[3]);
           }
           if(limbIndex == 2) {
             //Link 3
             inv[0] = Math.toDegrees(theta_1);
             inv[1] = Math.toDegrees(theta2_2);
             inv[2] = Math.toDegrees(-theta3_1);
             inv[3] = Math.toDegrees(theta4_2);
             System.out.println(inv[0]);
             System.out.println(inv[1]);
             System.out.println(inv[2]);
             System.out.println(inv[3]);
            }

            if(Double.isNaN(inv[0]) || Double.isNaN(inv[1]) || Double.isNaN(inv[2]) || Double.isNaN(inv[3]))
            	throw new ArithmeticException("Can't move to that position");
            else
            	return inv;


            	
        } catch (Throwable t) {
            BowlerStudio.printStackTrace(t);
            return null;
        }
	}

}

if(args==null)
	args=[0]
return new scriptJavaIKModel (args[0])