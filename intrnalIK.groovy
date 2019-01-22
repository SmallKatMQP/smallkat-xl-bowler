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
                                      double[] jointSpaceVector, DHChain chain) {
        println("My IK")
        try {
            ArrayList<DHLink> links = chain.getLinks();
            // THis is the jacobian for the given configuration
            //Matrix jacobian =  chain.getJacobian(jointSpaceVector);
            Matrix taskSpacMatrix = target.getMatrixTransform();

            int linkNum = jointSpaceVector.length;

            double x = target.getX();
            double y = target.getY();
            double z = target.getZ();

		double ang = 0;


            println(target.getRotationMatrixArray().toString());

            double l1_d = links.get(0).getD();
            double l2_d = links.get(1).getD();
            double l3_d = links.get(2).getD();
            double l4_d = links.get(3).getD();

            double[] inv = new double[linkNum];

            double theta1 = Math.atan(y / Math.abs(z));

            double r1 = Math.sqrt(Math.pow(z, 2) + Math.pow(y,2));
            double x1 = r1;
            double y1 = x;
            double Px = x1 - l4_d * Math.sin(ang);
            double Py = y1 - l4_d * Math.cos(ang);
            // Make below negative to switch to other angle
            double theta3_1 = Math.acos(((Math.pow(Px, 2) + Math.pow(Py,2)) - (Math.pow(l2_d,2) + Math.pow(l3_d, 2))) / (2 * l2_d * l3_d));
            double theta3_2 = theta3_1 * -1;

            double B = Math.atan(Py / Px);
            double Y = Math.acos((Math.pow(Px, 2) + Math.pow(Py, 2) + Math.pow(l2_d, 2) - Math.pow(l3_d, 2)) / (2 * l2_d * Math.sqrt(Math.pow(Px, 2) + Math.pow(Py, 2))));

            double theta2_1 = B + Y;
            double theta2_2 = B - Y;

            theta4_1 = (Math.PI / 2 - ang) - (theta2_1 + theta3_1);
            theta4_2 = (Math.PI / 2 - ang) - (theta2_2 + theta3_2);

		println(theta1);
		println(theta2_1);
		println(theta3_1);
		println(theta4_1);
		
			
            inv[0] = 0;//theta1;
            inv[1] = 0;//theta2_1;
            inv[2] = 0;//theta3_1;
            inv[3] = 0;//theta3_1;

            println("\r\n\r\nJoint Vector = " + inv + "\r\n\r\n");

            return inv;
        } catch (Throwable t) {
            BowlerStudio.printStackTrace(t)
        }
    }
};
