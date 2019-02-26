import java.util.ArrayList;

import com.neuronrobotics.sdk.addons.kinematics.DHChain;
import com.neuronrobotics.sdk.addons.kinematics.DHLink;
import com.neuronrobotics.sdk.addons.kinematics.DhInverseSolver;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import com.neuronrobotics.sdk.common.Log;
import Jama.Matrix;

return new DhInverseSolver() {

	@Override
	public double[] inverseKinematics(TransformNR target, double[] jointSpaceVector, DHChain chain ) {

    int linkNum = jointSpaceVector.length;
    double [] inv = new double[linkNum];

    ArrayList<DHLink> links = chain.getLinks();

    double xTarget = target.getX();
    double yTarget = target.getY();
    double zTarget = target.getZ();
    
    l1_r = links.get(0).getR();
    l2_r = links.get(1).getR();


    inv[0] = 0;
    inv[1] = 0;
    inv[2] = 0;

    if(Double.isNaN(inv[0]) || Double.isNaN(inv[1]) || Double.isNaN(inv[2]) || Double.isNaN(inv[3])){
        throw new ArithmeticException("Can't move to that position");
    } else {
        return inv;
    }
	}
};
