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

// TRUE to render the model, FALSE to render a stick figure
boolean isFullModel = false;

return new ICadGenerator(){

	private CSG moveDHValues(CSG incoming,DHLink dh ){
		TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
		Transform move = TransformFactory.nrToCSG(step)
		return incoming.transformed(move)

	}

	@Override
	public ArrayList<CSG> generateBody(MobileBase b ) {
		ArrayList<CSG> allCad=new ArrayList<>();
		File mainBodyFile = ScriptingEngine.fileFromGit("https://github.com/SmallKatMQP/smallkat-xl-bowler.git", "cad/Body.stl");

		// Load the .CSG from the disk and cache it in memory
		CSG body = new Cube(320,		// x dimension
												 50,		// y dimension
												 30)		// z dimension  
												 .toCSG()
												 .movex(15)
												 .movez(20);
		if (isFullModel){
			body  = Vitamins.get(mainBodyFile);
		}

		body.setManipulator(b.getRootListener());
		body.setColor(javafx.scene.paint.Color.WHITE)
		def parts = [body ] as ArrayList<CSG>
		for(int i=0;i<parts.size();i++){
			parts.get(i).setColor(javafx.scene.paint.Color.GRAY)
		}
		return parts;
	}

	@Override
	public ArrayList<CSG> generateCad(DHParameterKinematics dhParams, int linkIndex){
		ArrayList<CSG> allCad=new ArrayList<>();
		String limbName = dhParams.getScriptingName()
		File legFile = null

		if(limbName.contentEquals("Tail") && isFullModel){
		  if(linkIndex ==0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/TailJoint.stl");
		  } else if(linkIndex ==1){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Tail.stl");
		  } else {
		    println "Incorrect Tail Index"
		  }
		} else if(limbName.contentEquals("Head") && isFullModel){
		  if(linkIndex >2)
		    return allCad;
		  if(linkIndex ==0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/HeadLink.stl");
		  } else if(linkIndex ==1){
		    legFile = ScriptingEngine.fileFromGit(giturl,"cad/HeadNeck.stl");
		  } else if (linkIndex == 2){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Head.stl");
		  } else {
		    print "Incorrect Head Index: "
		    println linkIndex
		  }
		} else if (limbName.contentEquals("FrontLeft") && isFullModel){
			println "FrontLeft"
		  if(linkIndex == 0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Shoulder_FL2.stl");
		  } else if(linkIndex == 1){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/TopLeg_FL2.stl");
		  } else if(linkIndex == 2){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/MidLeg_Left2.stl");
		  } else if (linkIndex == 3){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Foot_FL2.stl");
		  } else {
		    print "Incorrect FrontLeft Leg Index: "
		    println linkIndex
		  }
		} else if (limbName.contentEquals("FrontRight") && isFullModel){
			println "FrontRight"
		  if(linkIndex == 0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Shoulder_FR2.stl");
		  } else if(linkIndex == 1){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/TopLeg_FR2.stl");
		  } else if(linkIndex == 2){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/MidLeg_Right2.stl");
		  } else if (linkIndex == 3){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Foot_FR2.stl");
		  } else {
		    print "Incorrect FrontRight Leg Index: "
		    println linkIndex
		  }
		} else if (limbName.contentEquals("BackLeft") && isFullModel){
			println "BackLeft"
		  if(linkIndex == 0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Shoulder_BL2.stl");
		  } else if(linkIndex == 1){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/TopLeg_BL2.stl");
		  } else if(linkIndex == 2){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/MidLeg_Left2.stl");
		  } else if (linkIndex == 3){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Foot_BL2.stl");
		  } else {
		    print "Incorrect BackLeft Leg Index: "
		    println linkIndex
		  }
		} else if (limbName.contentEquals("BackRight") && isFullModel){
			println "BackRight"
		  if(linkIndex == 0){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Shoulder_BR2.stl");
		  } else if(linkIndex == 1){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/TopLeg_BR2.stl");
		  } else if(linkIndex == 2){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/MidLeg_Right2.stl");
		  } else if (linkIndex == 3){
		    legFile = ScriptingEngine.fileFromGit(giturl, "cad/Foot_BR2.stl");
		  } else {
		    print "Incorrect BackRight Leg Index: "
		    println linkIndex
		  }
		} else {
		  print "Unknown limbName:"
		  println limbName
		}

		// Load the .CSG from the disk and cache it in memory
		println "Loading " +legFile
		
		if (legFile == null){
			// Return the DH Parameter Viewer
			return generateStickModel(dhParams, linkIndex)
		} else {
			// Return the regular viewer
			ArrayList<DHLink> dhLinks = dhParams.getChain().getLinks()
			DHLink dh = dhLinks.get(linkIndex)
			// Hardware to engineering units configuration
			LinkConfiguration conf = dhParams.getLinkConfiguration(linkIndex);
			// Engineering units to kinematics link (limits and hardware type abstraction)
			AbstractLink abstractLink = dhParams.getAbstractLink(linkIndex);// Transform used by the UI to render the location of the object
			// Transform used by the UI to render the location of the object
			Affine manipulator = dh.getListener();

			CSG body = Vitamins.get(legFile).setManipulator(manipulator);

			def parts = [body ] as ArrayList<CSG>
			for(int i=0;i<parts.size();i++){
				parts.get(i).setColor(javafx.scene.paint.Color.RED)
			}
			return parts
		}
	}

	private ArrayList<CSG> generateStickModel(DHParameterKinematics d, int linkIndex){
ArrayList<CSG> allCad=new ArrayList<>();
		ArrayList<DHLink> dhLinks = d.getChain().getLinks()
		DHLink dh = dhLinks.get(linkIndex)
		// Hardware to engineering units configuration
		LinkConfiguration conf = d.getLinkConfiguration(linkIndex);
		// Engineering units to kinematics link (limits and hardware type abstraction)
		AbstractLink abstractLink = d.getAbstractLink(linkIndex);// Transform used by the UI to render the location of the object
		// Transform used by the UI to render the location of the object
		Affine manipulator = dh.getListener();
		Affine lastLinkFrame
		def massKg = conf.getMassKg()
		def centerOfMass = TransformFactory.nrToCSG(conf.getCenterOfMassFromCentroid() )
		def CMvis = new Sphere(50*massKg).toCSG()
					.transformed(centerOfMass)
		
		if(linkIndex==0)
			lastLinkFrame=d.getRootListener()
		else
			lastLinkFrame=dhLinks.get(linkIndex-1).getListener();
		
		def rVal = new Cube(dh.getR()>0?dh.getR():5,1,1).toCSG()
					.toXMax()
					.toZMax()
		rVal.setColor(javafx.scene.paint.Color.RED)
		CSG profile = new Cube(1,// x dimention
					20,// y dimention
				
				1//  Z dimention
				)
				.toCSG()// converts it to a CSG tor display
				.toYMin()
				.toZMin()
				//.toXMin()
				
		CSG theta;
		double thetaval = Math.toDegrees(dh.getTheta())
		if(Math.abs(thetaval)>10){
			theta= CSG.unionAll(
			Extrude.revolve(profile,
						0, // rotation center radius, if 0 it is a circle, larger is a donut. Note it can be negative too
						Math.abs(thetaval),// degrees through wich it should sweep
						(int)10)//number of sweep increments
			)
		}else{
			theta = profile
		}
		
		if(thetaval>0){
			theta= theta.rotz(-thetaval)
		}
		theta= theta.rotz(90)
		.movez(-0.5)
		theta.setColor(thetaval>0?javafx.scene.paint.Color.BLUE:javafx.scene.paint.Color.AQUA)

		CSG alpha;
		double alphaVal = Math.toDegrees(dh.getAlpha())
		if(Math.abs(alphaVal)>10){
			alpha= CSG.unionAll(
			Extrude.revolve(profile,
						0, // rotation center radius, if 0 it is a circle, larger is a donut. Note it can be negative too
						Math.abs(alphaVal),// degrees through wich it should sweep
						(int)10)//number of sweep increments
			)
			//.rotz(alphaVal<0?-alphaVal:0)
		}else{
			alpha = profile
		}
		alpha= alpha.roty(90)
		if(alphaVal>0){
			alpha= alpha.rotx(alphaVal)	
		}
		alpha= alpha
			.rotx(-90)
			.movex(-dh.getR())
		alpha.setColor(alphaVal>0?javafx.scene.paint.Color.YELLOW:javafx.scene.paint.Color.GOLDENROD)

		def dpart = new Cube(1,1,dh.getD()>0?dh.getD():1).toCSG()
					.toZMin()
		double upperLimit = -abstractLink.getMaxEngineeringUnits()
		double lowerLimit = -abstractLink.getMinEngineeringUnits()
		double totalRange = -(upperLimit-lowerLimit)
		def min = 10
		if(totalRange>360-min)
			totalRange=360-min
		if(totalRange<min)
			totalRange=min
		def name = d.getScriptingName()
		//println name
		
		def printit = name.equals("FrontRight")&&linkIndex==0
		if(printit)println "\n\n\nLink range = "+totalRange+" "+upperLimit+" " +lowerLimit
		def rangeComp = 360-totalRange
		def orentationAdjust = -thetaval+90
		def Range
		if(rangeComp>min)
			Range = CSG.unionAll(
			Extrude.revolve(profile,
					0, // rotation center radius, if 0 it is a circle, larger is a donut. Note it can be negative too
					rangeComp,// degrees through wich it should sweep
					(int)min)//number of sweep increments
			)
		else
			Range =profile
		
		Range=Range
			.rotz(lowerLimit+orentationAdjust)
			.movez(-2)

		
		Range.setColor(javafx.scene.paint.Color.LIGHTGREEN)
		def upperLim = profile
					.rotz(upperLimit+orentationAdjust)
					.setColor(javafx.scene.paint.Color.HOTPINK)
		def lowerLim = profile
					.rotz(lowerLimit+orentationAdjust)
					.setColor(javafx.scene.paint.Color.WHITE)
		def zeroLim = profile
					.rotz(orentationAdjust)
					.setColor(javafx.scene.paint.Color.INDIGO)
		def lastFrameParts = [theta,dpart,upperLim,lowerLim,zeroLim,Range]
		def parts = [rVal,alpha,CMvis] as ArrayList<CSG>
		for(int i=0;i<parts.size();i++){
			parts.get(i).setManipulator(manipulator);
			//parts.get(i).setColor(javafx.scene.paint.Color.RED)
		}
		for(int i=0;i<lastFrameParts.size();i++){
			lastFrameParts.get(i).setManipulator(lastLinkFrame);
			//parts.get(i).setColor(javafx.scene.paint.Color.RED)
		}
		parts.addAll(lastFrameParts)
		return parts;

	}
}
