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

    int linkNum = jointSpaceVector.length;
    double [] inv = new double[linkNum];

    inv[0] = 0;
    inv[1] = 0;
    inv[2] = 0;
    
		return inv;
	}
};
