import com.neuronrobotics.bowlerstudio.creature.ICadGenerator;
import com.neuronrobotics.bowlerstudio.creature.CreatureLab;
import org.apache.commons.io.IOUtils;
import com.neuronrobotics.bowlerstudio.vitamins.*;
import java.nio.file.Paths;
import eu.mihosoft.vrl.v3d.FileUtil;
import eu.mihosoft.vrl.v3d.Transform;
import javafx.scene.transform.Affine;
import com.neuronrobotics.bowlerstudio.physics.TransformFactory;
println "Loading STL file"
// Load an STL file from a git repo
// Loading a local file also works here
String giturl = "https://github.com/SmallKatMQP/smallkat-xl-bowler.git";
return new ICadGenerator(){

	private CSG moveDHValues(CSG incoming,DHLink dh ){
		TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
		Transform move = TransformFactory.nrToCSG(step)
		return incoming.transformed(move)

	}

	@Override
	public ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {
		ArrayList<CSG> allCad=new ArrayList<>();
		String limbName = d.getScriptingName()
		File legFile = null
		// boolean mirror=true
		// if(limbName.contentEquals("DefaultLeg3")||limbName.contentEquals("DefaultLeg4")){
		// 	println "Mirror leg parts"
		// 	mirror=false
		// }
		TransformNR  legRoot= d.getRobotToFiducialTransform()
		def leftSide=false
		def rear = true
		if(legRoot.getY()>0){
			leftSide=true;
		}
		if(legRoot.getX()>0){
			rear=false;
		}

		if(limbName.contentEquals("Tail")){
		  if(linkIndex ==0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/TailJoint.stl");
		  } else if(linkIndex ==1){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Tail.stl");
		  } else if (linkIndex == 2) {
		    reteurn allCad;
		  } else {
		    println "Incorrect Tail Index"
		  }
		} else if(limbName.contentEquals("Head")){
		  if(linkIndex >1)
		    return allCad;
		  if(linkIndex ==0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/HeadLink.stl");
		  } else if(linkIndex ==1){
		    legFile = ScriptingEngine.fileFromGit(giturl,"cad/HeadNeck.stl");
		  } else if (linkIndex == 2){
		    // legFile = ScriptingEngine.fileFromGit(giturl, "cad/Head.stl");
				legFile= new Cube(5).toCSG();
		  } else if(linkIndex > 2){
		    return allCad;
		  } else {
		    print "Incorrect Head Index: "
		    println linkIndex
		  }
		} else if (limbName.contentEquals("FrontLeft")){
			println "FrontLeft"
		  if(linkIndex == 0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Shoulder_FL_BR.stl");
		  } else if(linkIndex == 1){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/TopLeg_FL_BR.stl");
		  } else if(linkIndex == 2){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/MidLeg_Left.stl");
		  } else if (linkIndex == 3){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Foot_FL.stl");
		  } else {
		    print "Incorrect FrontLeft Leg Index: "
		    println linkIndex
		  }
		} else if (limbName.contentEquals("FrontRight")){
			println "FrontRight"
		  if(linkIndex == 0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Shoulder_FR_BL.stl");
		  } else if(linkIndex == 1){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/TopLeg_FR_BL.stl");
		  } else if(linkIndex == 2){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/MidLeg_Right.stl");
		  } else if (linkIndex == 3){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Foot_FR.stl");
		  } else {
		    print "Incorrect FrontLeft Leg Index: "
		    println linkIndex
		  }
		} else if (limbName.contentEquals("BackLeft")){
			println "BackLeft"
		  if(linkIndex == 0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Shoulder_FR_BL.stl");
		  } else if(linkIndex == 1){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/TopLeg_FR_BL.stl");
		  } else if(linkIndex == 2){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/MidLeg_Left.stl");
		  } else if (linkIndex == 3){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Foot_BL.stl");
		  } else {
		    print "Incorrect FrontLeft Leg Index: "
		    println linkIndex
		  }
		} else if (limbName.contentEquals("BackRight")){
			println "BackRight"
		  if(linkIndex == 0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Shoulder_FL_BR.stl");
		  } else if(linkIndex == 1){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/TopLeg_FL_BR.stl");
		  } else if(linkIndex == 2){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/MidLeg_Right.stl");
		  } else if (linkIndex == 3){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Foot_FR.stl");
		  } else {
		    print "Incorrect FrontLeft Leg Index: "
		    println linkIndex
		  }
		} else {
		  print "Unknown limbName:"
		  println limbName
		}

		ArrayList<DHLink> dhLinks = d.getChain().getLinks()
		DHLink dh = dhLinks.get(linkIndex)
		// Hardware to engineering units configuration
		LinkConfiguration conf = d.getLinkConfiguration(linkIndex);
		// Engineering units to kinematics link (limits and hardware type abstraction)
		AbstractLink abstractLink = d.getAbstractLink(linkIndex);// Transform used by the UI to render the location of the object
		// Transform used by the UI to render the location of the object
		Affine manipulator = dh.getListener();

		// Load the .CSG from the disk and cache it in memory
		println "Loading " +legFile
		CSG body  = Vitamins.get(legFile)
		body = moveDHValues(body,dh)

		// if(linkIndex ==0){
		// 	// body=moveDHValues(body,dh)
		// 	if(limbName.contentEquals("Head")||limbName.contentEquals("Tail")){
		// 		// body=body
		// 		// // .rotz(90)
		// 		// // .rotx(180)
		// 		// .movex(-41)
		// 		// .movey(-21)
		// 		// .movez(-22)
		// 			//.movez(-11.5)
		// 	}	else{
		// 		// body=body.roty(180)
		// 		// .rotx(180)
		// 		//if(rear)
		// 			//body=body.rotx(180)
		// 	}
		//
		// }
		// if(linkIndex ==1){
		// 	if(limbName.contentEquals("Head")){
		// 		body=body.roty(180).movex(50).movey(-18).movez(-38.5)
		// 	}else if(limbName.contentEquals("Tail")){
		// 		body=body.roty(180).rotz(-90).movey(125)
		// 	}else{
		// 		// body=body.roty(180)
		// 	}
		// }
		// if(linkIndex ==2){
		// 	body=body.roty(180)
		// }

		body.setManipulator(manipulator);

		def parts = [body ] as ArrayList<CSG>
		for(int i=0;i<parts.size();i++){
			parts.get(i).setColor(javafx.scene.paint.Color.RED)
		}
		return parts;

	}


	@Override
	public ArrayList<CSG> generateBody(MobileBase b ) {
		ArrayList<CSG> allCad=new ArrayList<>();
		File mainBodyFile = ScriptingEngine.fileFromGit("https://github.com/SmallKatMQP/smallkat-xl-bowler.git", "cad/Body.stl");

		// Load the .CSG from the disk and cache it in memory
		CSG body  = Vitamins.get(mainBodyFile)

		body.setManipulator(b.getRootListener());
		body.setColor(javafx.scene.paint.Color.WHITE)
		def parts = [body ] as ArrayList<CSG>
		for(int i=0;i<parts.size();i++){
			parts.get(i).setColor(javafx.scene.paint.Color.GRAY)
		}
		return parts;
	}
}
