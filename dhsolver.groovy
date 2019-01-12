import java.util.ArrayList;

import com.neuronrobotics.sdk.addons.kinematics.DHChain;
import com.neuronrobotics.sdk.addons.kinematics.DHLink;
import com.neuronrobotics.sdk.addons.kinematics.DhInverseSolver;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import com.neuronrobotics.sdk.common.Log;					
import Jama.Matrix;

return new DhInverseSolver() {
	
	@Override
	public double[] inverseKinematics(TransformNR target,
			double[] jointSpaceVector,DHChain chain ) {
		ArrayList<DHLink> links = chain.getLinks();
		// THis is the jacobian for the given configuration
		//Matrix jacobian =  chain.getJacobian(jointSpaceVector);
		Matrix taskSpacMatrix = target.getMatrixTransform();
		
		int linkNum = jointSpaceVector.length;

		double x = target.getX();
		double y = target.getY();
		double z = target.getZ();
		double ang = target.getRotation();

		double l1_d = links.get(0).getD();
		double l2_d = links.get(1).getD();
		double l3_d = links.get(2).getD();
		double l4_d = links.get(3).getd();
		
		double [] inv = new double[linkNum];

		double theta1 = Math.atan2(y/Math.abs(z));

		double r1 = Math.sqrt(z^2 + y^2);
		double x1 = r1;
		double y1 = x;
		double Px = x1 - l4_d * Math.sin(ang);
		double Py = y1 - l4_d * Math.cos(ang);
		// Make below negative to switch to other angle
		double theta3_1 = Math.acos(((Px^2 + Py^2)- (l2_d^2 + l3_d^2))/(2*l2_d*l3_d));
		double theta3_2 = theta3_1 * -1;

		double B = Math.atan(Py/Px);
		double Y = Math.acos((Px^2 + Py^2 + l2_d^2 - l3_d^2) / (2*l2_d * Math.sqrt(Px^2 + Py^2)));

		double theta2_1 = B+Y;
		double theta2_2 = B-Y;

		theta4_1 = (Math.PI/2 - ang) - (theta2_1 + theta3_1);
		theta4_2 = (Math.PI/2 - ang) - (theta2_2 + theta3_2);

		inv[0] = theta1;
		inv[1] = theta2_1;
		inv[2] = theta3_1;
		inv[3] = theta3_1;

		return inv;
	}
};
