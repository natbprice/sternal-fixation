
# -----------------------------------------------------------------------------
#
# Python code for the Bone and Screw model
#   (parametrized for Abaqus CAE/Standard)
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------   
# To run the Python 
#
#     (make sure the Work Directory is set to the directory were this file is located )
#
#     At Abaqus command window,  type 
#      >>abaqus cae script=***.py
#       or
#     At Abaqus CAE menu, 
#     File -> Run Script
#       or
#     At Aabqus CAE command line,     
#     >>execfile('***.py')

# *****************************************************************************
# Import modules required for CAE and Python (No additional files requred)
# *****************************************************************************

from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
from math import *
from Numeric import *

# ***************************************************************************** 
# Create a list of  'simulation properties' parameters
# *****************************************************************************

# Displacement Load (mm)
DispLoad=0.025

# Friction Formulation for Tangential Contact Interaction (Lagrange, Coulomb, or Rough)
contactForm='Rough'

# Friction factor (Lagrange or Coulomb ONLY)
fricFact=2

# Bone Strength (Low, Med, or High)
BoneStrength='Med'

# Global Mesh Size
meshSize=0.65

# *****************************************************************************
# Create a list of 'mechanical properties' parameters
# *****************************************************************************

Eplate=104.1e3           	# Young's Modulus Plate (N/mm^2)
Escrew=112e3            	# Young's Modulus Screw (N/mm^2)
n=0.3         			# Poisson's Ratio

if BoneStrength=='Low':
	Ecortical=6e3		# Young's Modulus Cortical Bone (N/mm^2)
	Etrabecular=0.04e3       # Young's Modulus Trabecular Bone (N/mm^2)
	
if BoneStrength=='Med':
	Ecortical=12e3		# Young's Modulus Cortical Bone (N/mm^2)
	Etrabecular=1.1e3       # Young's Modulus Trabecular Bone (N/mm^2)
	
if BoneStrength=='High':
	Ecortical=25e3		# Young's Modulus Cortical Bone (N/mm^2)
	Etrabecular=2.2e3       # Young's Modulus Trabecular Bone (N/mm^2)

# ***************************************************************************** 
# Create a list of  'geometrical properties' parameters
# *****************************************************************************

# Coordinates of Screw Hole 1 (cx,cy)
cx=5.715
cy=0

# Coordinates Screw Hole 2 (cx2,cy2)
cx2=8.285
cy2=7.04

# Radius of Screw Hole (R)
R=2.69/2

# Bone Thickness (dbone)
dbone=12

# Cortical Thickness (dcort)
dcort=0.75

# Trabecular Thickness (dtrab)
dtrab=dbone-2*dcort

# Plate Thickness(dplate)
dplate=1.6

# Length of Screw (dscrew)
dscrew=10.775

# ***************************************************************************** 
# Create model & assembly
# *****************************************************************************

model_name='Bone and Screw'
myModel=mdb.Model(name=model_name)
myAssem=myModel.rootAssembly

# *****************************************************************************
# Materials
# *****************************************************************************
myModel.Material(name='Pure TI Grade IV')
myModel.materials['Pure TI Grade IV'].Elastic(table=((
   Eplate, n), ))
myModel.Material(name='Ti-6AL-4V')
myModel.materials['Ti-6AL-4V'].Elastic(table=((
   Escrew, n), ))
myModel.Material(name='Cortical Bone')
myModel.materials['Cortical Bone'].Elastic(table=((
   Ecortical, n), ))
myModel.Material(name='Trabecular Bone')
myModel.materials['Trabecular Bone'].Elastic(table=((
   Etrabecular, n), ))

# ***************************************************************************** 
# Create Sections
# *****************************************************************************
myModel.HomogeneousSolidSection(material='Pure TI Grade IV', 
    name='Plate', thickness=None)
myModel.HomogeneousSolidSection(material='Ti-6AL-4V', 
    name='Screw', thickness=None)
myModel.HomogeneousSolidSection(material='Cortical Bone', 
    name='Cortical', thickness=None)
myModel.HomogeneousSolidSection(material='Trabecular Bone', 
    name='Trabecular', thickness=None)

# *****************************************************************************
# Define Function to Construct Shell Part for Partitioning Bone
# *****************************************************************************                    
def createPartitionBone(myModel, myAssem):

	# ================= Create Section Sketch ==============================
	BonePartitionSketch=myModel.ConstrainedSketch(name='Bone Partition Sketch',sheetSize=10.0)
	
	# ================= Draw Sketch ========================================
	BonePartitionSketch.rectangle(point1=(0.0, 9.265),point2=(14.96, -5.715))
	BonePartitionSketch.rectangle(point1=(3.49, 2.225),point2=(7.94, -2.225))
	BonePartitionSketch.rectangle(point1=(6.06, 9.265),point2=(10.51, 4.815))
	
	BonePartitionSketch.Line(point1=(3.49,-2.225),point2=(3.49,-5.715))
	BonePartitionSketch.Line(point1=(7.94,-2.225),point2=(7.94,-5.715))
		
	rp=R*cos(pi/4)
	
	BonePartitionSketch.CircleByCenterPerimeter(center=(cx,cy), point1=(cx+rp,cy+rp))
	BonePartitionSketch.CircleByCenterPerimeter(center=(cx2,cy2), point1=(cx2+rp,cy2+rp))
	
	BonePartitionSketch.Line(point1=(3.49,2.225),point2=(cx-rp,cy+rp))
	BonePartitionSketch.Line(point1=(7.94,2.225),point2=(cx+rp,cy+rp))
	BonePartitionSketch.Line(point1=(3.49,-2.225),point2=(cx-rp,cy-rp))
	BonePartitionSketch.Line(point1=(7.94,-2.225),point2=(cx+rp,cy-rp))
	
	BonePartitionSketch.Line(point1=(6.06,9.265),point2=(cx2-rp,cy2+rp))
	BonePartitionSketch.Line(point1=(10.51,9.265),point2=(cx2+rp,cy2+rp))
	BonePartitionSketch.Line(point1=(6.06,4.815),point2=(cx2-rp,cy2-rp))
	BonePartitionSketch.Line(point1=(10.51,4.815),point2=(cx2+rp,cy2-rp))
	
	BonePartitionSketch.Line(point1=(3.49,2.225),point2=(6.06,4.815))
	BonePartitionSketch.Line(point1=(7.94,2.225),point2=(10.51,4.815))
	
	BonePartitionSketch.Line(point1=(0,-2.225),point2=(14.96,-2.225))
	BonePartitionSketch.Line(point1=(0,2.225),point2=(14.96,2.225))
	BonePartitionSketch.Line(point1=(0,4.815),point2=(14.96,4.815))
	
	# ================= Create Part ========================================
	BonePartitionPart=myModel.Part(dimensionality=THREE_D, name='Bone Partition', type=
	    DEFORMABLE_BODY)
	BonePartitionPart.BaseShellExtrude(depth=dbone, sketch=BonePartitionSketch)
	
	myAssem.Instance(dependent=ON, name='Bone Partition Part', part=BonePartitionPart)

# *****************************************************************************
# Define Function to Construct Bone Part
# *****************************************************************************                    
def createPartBone(myModel, myAssem):

	# ================= Create Bone Section Sketches =======================
	BoneSketch=myModel.ConstrainedSketch(name='Bone Sketch',sheetSize=10.0)
	
	# ================= Draw Sketch ========================================
	BoneSketch.rectangle(point1=(0.0, 9.265),point2=(14.96, -5.715))

	# ================= Create Parts =======================================
	BonePart=myModel.Part(dimensionality=THREE_D, name='Solid Bone',
		type=DEFORMABLE_BODY)
	BonePart.BaseSolidExtrude(sketch=BoneSketch, depth=dbone)
	
	myAssem.Instance(dependent=ON, name='Solid Bone', part=BonePart)
	
	# ================= Partition Bone Layers ==============================
	face1 = BonePart.faces.findAt((0,0,0.))
	BonePart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dcort)
	BonePart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dcort+dtrab)
	
	# ================= Assign Sections ====================================
	d1 = BonePart.datums
	pickedCells = BonePart.cells
	BonePart.PartitionCellByDatumPlane(datumPlane=d1[2],
	    cells=pickedCells)
	pickedCells = BonePart.cells
	BonePart.PartitionCellByDatumPlane(datumPlane=d1[3],
	    cells=pickedCells)
	
	pickedCells = BonePart.cells.findAt(((0,9.265,0),),
		((14.96,9.265,0),), ((14.96,-5.715,0),),)
	BonePart.Set(cells=pickedCells, name='Bottom Cortical')
	BonePart.SectionAssignment(offset=0.0, 
	    offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
	    cells=pickedCells), sectionName='Cortical', thicknessAssignment=
	    FROM_SECTION)

	pickedCells = BonePart.cells.findAt(((0,9.265,12),),
		((14.96,9.265,12),), ((14.96,-5.715,12),),)
	BonePart.Set(cells=pickedCells, name='Top Cortical')
	BonePart.SectionAssignment(offset=0.0, 
	    offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
	    cells=pickedCells), sectionName='Cortical', thicknessAssignment=
	    FROM_SECTION)
	
	pickedCells = BonePart.cells.findAt(((0,9.265,dtrab),),
		((14.96,9.265,dtrab),), ((14.96,-5.715,dtrab),),)
	BonePart.Set(cells=pickedCells, name='Trabecular')
	BonePart.SectionAssignment(offset=0.0, 
	    offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
	    cells=pickedCells), sectionName='Trabecular', thicknessAssignment=
	    FROM_SECTION)

# *****************************************************************************
# Define Function to Construct Shell Part for Partitioning Plate
# *****************************************************************************                    
def createPartitionPlate(myModel, myAssem):
	# ================= Create Plate Section Sketches =======================
	PlatePartitionSketch=myModel.ConstrainedSketch(name='Plate Sketch',sheetSize=10.0)
	
	# ================= Draw Sketch ========================================
	PlatePartitionSketch.rectangle(point1=( 0.0, 1.195),point2=(2.46, -1.195))
	PlatePartitionSketch.rectangle(point1=(3.49, 2.225),point2=(7.94, -2.225))
	PlatePartitionSketch.rectangle(point1=(4.52, -3.255),point2=(6.91, -5.715))
	PlatePartitionSketch.rectangle(point1=(6.06, 9.265),point2=(10.51, 4.815))
	
	PlatePartitionSketch.Line(point1=(2.46,1.195),point2=(3.49,2.225))
	PlatePartitionSketch.Line(point1=(2.46,-1.195),point2=(3.49,-2.225))
	PlatePartitionSketch.Line(point1=(3.49,-2.225),point2=(4.52,-3.255))
	PlatePartitionSketch.Line(point1=(7.94,-2.225),point2=(6.91,-3.255))
	PlatePartitionSketch.Line(point1=(3.49,2.225),point2=(6.06,4.815))
	PlatePartitionSketch.Line(point1=(7.94,2.225),point2=(10.51,4.815))
	
	rp=R*cos(pi/4)
	
	PlatePartitionSketch.CircleByCenterPerimeter(center=(cx,cy), point1=(cx+rp,cy+rp))
	PlatePartitionSketch.CircleByCenterPerimeter(center=(cx2,cy2), point1=(cx2+rp,cy2+rp))
	
	PlatePartitionSketch.Line(point1=(3.49,2.225),point2=(cx-rp,cy+rp))
	PlatePartitionSketch.Line(point1=(7.94,2.225),point2=(cx+rp,cy+rp))
	PlatePartitionSketch.Line(point1=(3.49,-2.225),point2=(cx-rp,cy-rp))
	PlatePartitionSketch.Line(point1=(7.94,-2.225),point2=(cx+rp,cy-rp))
	
	PlatePartitionSketch.Line(point1=(6.06,9.265),point2=(cx2-rp,cy2+rp))
	PlatePartitionSketch.Line(point1=(10.51,9.265),point2=(cx2+rp,cy2+rp))
	PlatePartitionSketch.Line(point1=(6.06,4.815),point2=(cx2-rp,cy2-rp))
	PlatePartitionSketch.Line(point1=(10.51,4.815),point2=(cx2+rp,cy2-rp))

	# ================= Create Parts =======================================
	PlatePartitionPart=myModel.Part(dimensionality=THREE_D, name='Plate Partition', type=
	    DEFORMABLE_BODY)
	PlatePartitionPart.BaseShellExtrude(depth=dplate, sketch=PlatePartitionSketch)
	
	myAssem.Instance(dependent=ON, name='Plate Partition Part', part=PlatePartitionPart)
	
	myAssem.translate(instanceList=('Plate Partition Part',), 
		vector=(0,0,dbone))
	
# *****************************************************************************
# Define Function to Construct Plate Part
# *****************************************************************************                    
def createPartPlate(myModel, myAssem):
	# ================= Create Plate Section Sketches =======================
	PlateSketch=myModel.ConstrainedSketch(name='Plate Sketch',sheetSize=10.0)
	
	# ================= Draw Sketch ========================================
	PlateSketch.rectangle(point1=( 0.0, 1.195),point2=(2.46, -1.195))
	PlateSketch.rectangle(point1=(3.49, 2.225),point2=(7.94, -2.225))
	PlateSketch.rectangle(point1=(4.52, -3.255),point2=(6.91, -5.715))
	PlateSketch.rectangle(point1=(6.06, 9.265),point2=(10.51, 4.815))
	
	PlateSketch.Line(point1=(2.46,1.195),point2=(3.49,2.225))
	PlateSketch.Line(point1=(2.46,-1.195),point2=(3.49,-2.225))
	PlateSketch.Line(point1=(3.49,-2.225),point2=(4.52,-3.255))
	PlateSketch.Line(point1=(7.94,-2.225),point2=(6.91,-3.255))
	PlateSketch.Line(point1=(3.49,2.225),point2=(6.06,4.815))
	PlateSketch.Line(point1=(7.94,2.225),point2=(10.51,4.815))
	
	PlateSketch.delete(objectList=(PlateSketch.geometry.findAt((2.46,0)),))
	PlateSketch.delete(objectList=(PlateSketch.geometry.findAt((3.49,0)),))
	PlateSketch.delete(objectList=(PlateSketch.geometry.findAt((5.7,-3.255)),))
	PlateSketch.delete(objectList=(PlateSketch.geometry.findAt((5.7,-2.225)),))
	PlateSketch.delete(objectList=(PlateSketch.geometry.findAt((5.7,2.225)),))
	PlateSketch.delete(objectList=(PlateSketch.geometry.findAt((7.94,4.815)),))

	# ================= Create Parts =======================================
	PlatePart=myModel.Part(dimensionality=THREE_D, name='Solid Plate',
		type=DEFORMABLE_BODY)
	PlatePart.BaseSolidExtrude(sketch=PlateSketch, depth=dplate)
	
	myAssem.Instance(dependent=ON, name='Solid Plate', part=PlatePart)
	
	myAssem.translate(instanceList=('Solid Plate',), 
		vector=(0,0,dbone))
	
	# ================= Assign Section =====================================
	pickedCells=PlatePart.cells
	PlatePart.SectionAssignment(offset=0.0, 
	    offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
	    cells=pickedCells), sectionName='Plate', thicknessAssignment=
	    FROM_SECTION)
	
# *****************************************************************************
# Define Function to Construct Screw Parts 
# *****************************************************************************                    
def createPartScrew(myModel, myAssem):
	rp=R*cos(pi/4)

	# ================= Create Screw Section Sketch ========================
	ScrewSketch=myModel.ConstrainedSketch(name='Screw Sketch',sheetSize=10.0)
	ScrewSketch.CircleByCenterPerimeter(center=(cx,cy), point1=(cx+rp,cy+rp))
	
	ScrewSketch2=myModel.ConstrainedSketch(name='Screw Sketch 2',sheetSize=10.0)
	ScrewSketch2.CircleByCenterPerimeter(center=(cx2,cy2), point1=(cx2+rp,cy2+rp))

	# ================= Create Parts =======================================
	ScrewPart=myModel.Part(dimensionality=THREE_D, name='Screw 1',
		type=DEFORMABLE_BODY)
	ScrewPart.BaseSolidExtrude(sketch=ScrewSketch, depth=dscrew)
	
	ScrewPart2=myModel.Part(dimensionality=THREE_D, name='Screw 2',
		type=DEFORMABLE_BODY)
	ScrewPart2.BaseSolidExtrude(sketch=ScrewSketch2, depth=dscrew)
	
	# ================= Instance Part in Assembly ==========================
	myAssem.Instance(dependent=ON, name='Screw 1', part=ScrewPart)
	myAssem.translate(instanceList=('Screw 1',), vector=(0,0,dbone+dplate-dscrew))
	
	myAssem.Instance(dependent=ON, name='Screw 2', part=ScrewPart2)
	myAssem.translate(instanceList=('Screw 2',), vector=(0,0,dbone+dplate-dscrew))
	
	# ================= Assign Section =====================================
	pickedCells=ScrewPart.cells
	ScrewPart.SectionAssignment(offset=0.0, 
	    offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
	    cells=pickedCells), sectionName='Screw', thicknessAssignment=
	    FROM_SECTION)
	
	pickedCells=ScrewPart2.cells
	ScrewPart2.SectionAssignment(offset=0.0, 
	    offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
	    cells=pickedCells), sectionName='Screw', thicknessAssignment=
	    FROM_SECTION)
	
	# ================= Partition Screw Using Datum Planes =================
	###### Case 1 - Screw only partially penetrates trabecular
	if dscrew<(dplate+dcort+dtrab):
		dxt = dplate+dcort+dtrab-dscrew
		
		face1 = ScrewPart.faces.findAt((cx,cy,0.))
		ScrewPart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dtrab-dxt)
		ScrewPart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dtrab-dxt+dcort)
		
		face2 = ScrewPart2.faces.findAt((cx2,cy2,0.))
		ScrewPart2.DatumPlaneByOffset(plane=face2,flip=SIDE2, offset=dtrab-dxt)
		ScrewPart2.DatumPlaneByOffset(plane=face2,flip=SIDE2, offset=dtrab-dxt+dcort)
		
		d1 = ScrewPart.datums
		pickedCells = ScrewPart.cells
		ScrewPart.PartitionCellByDatumPlane(datumPlane=d1[3],
		    cells=pickedCells)
		pickedCells = ScrewPart.cells
		ScrewPart.PartitionCellByDatumPlane(datumPlane=d1[4],
		    cells=pickedCells)
		
		d2 = ScrewPart2.datums
		pickedCells = ScrewPart2.cells
		ScrewPart2.PartitionCellByDatumPlane(datumPlane=d2[3],
		    cells=pickedCells)
		pickedCells = ScrewPart2.cells
		ScrewPart2.PartitionCellByDatumPlane(datumPlane=d2[4],
		    cells=pickedCells)
		
		#============= Define Surfaces =================================
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Plate')
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate-dcort/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Top Cort')
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate-dcort-(dtrab-dxt)/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Trab')
		
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Plate')
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate-dcort/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Top Cort')
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate-dcort-(dtrab-dxt)/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Trab')
	
	###### Case 2 - Screw fully penetrates trabecular but doesn't penetrate bottom cortical
	elif dscrew==(dplate+dcort+dtrab):
		face1 = ScrewPart.faces.findAt((cx,cy,0.))
		ScrewPart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dtrab)
		ScrewPart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dtrab+dcort)
		
		face2 = ScrewPart2.faces.findAt((cx2,cy2,0.))
		ScrewPart2.DatumPlaneByOffset(plane=face2,flip=SIDE2, offset=dtrab)
		ScrewPart2.DatumPlaneByOffset(plane=face2,flip=SIDE2, offset=dtrab+dcort)
		
		d1 = ScrewPart.datums
		pickedCells = ScrewPart.cells
		ScrewPart.PartitionCellByDatumPlane(datumPlane=d1[3],
		    cells=pickedCells)
		pickedCells = ScrewPart.cells
		ScrewPart.PartitionCellByDatumPlane(datumPlane=d1[4],
		    cells=pickedCells)
		
		d2 = ScrewPart2.datums
		pickedCells = ScrewPart2.cells
		ScrewPart2.PartitionCellByDatumPlane(datumPlane=d2[3],
		    cells=pickedCells)
		pickedCells = ScrewPart2.cells
		ScrewPart2.PartitionCellByDatumPlane(datumPlane=d2[4],
		    cells=pickedCells)
		
		#============= Define Surfaces =================================
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Plate')
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate-dcort/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Top Cort')
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate-dcort-dtrab/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Trab')
		
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Plate')
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate-dcort/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Top Cort')
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate-dcort-dtrab/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Trab')

	###### Case 3 - Screw fully penetrates trabecular and partially penetrates bottom cortical	
	elif dscrew>(dplate+dcort+dtrab) and dscrew<(dplate+dbone):
		dxt = dplate+dbone-dscrew
		
		face1 = ScrewPart.faces.findAt((cx,cy,0.))
		ScrewPart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dcort-dxt)
		ScrewPart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dcort-dxt+dtrab)
		ScrewPart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dcort-dxt+dtrab+dcort)
		
		face2 = ScrewPart2.faces.findAt((cx2,cy2,0.))
		ScrewPart2.DatumPlaneByOffset(plane=face2,flip=SIDE2, offset=dcort-dxt)
		ScrewPart2.DatumPlaneByOffset(plane=face2,flip=SIDE2, offset=dcort-dxt+dtrab)
		ScrewPart2.DatumPlaneByOffset(plane=face2,flip=SIDE2, offset=dcort-dxt+dtrab+dcort)
		
		d1 = ScrewPart.datums
		pickedCells = ScrewPart.cells
		ScrewPart.PartitionCellByDatumPlane(datumPlane=d1[3],
		    cells=pickedCells)
		pickedCells = ScrewPart.cells
		ScrewPart.PartitionCellByDatumPlane(datumPlane=d1[4],
		    cells=pickedCells)
		pickedCells = ScrewPart.cells
		ScrewPart.PartitionCellByDatumPlane(datumPlane=d1[5],
		    cells=pickedCells)
		
		d2 = ScrewPart2.datums
		pickedCells = ScrewPart2.cells
		ScrewPart2.PartitionCellByDatumPlane(datumPlane=d2[3],
		    cells=pickedCells)
		pickedCells = ScrewPart2.cells
		ScrewPart2.PartitionCellByDatumPlane(datumPlane=d2[4],
		    cells=pickedCells)
		pickedCells = ScrewPart2.cells
		ScrewPart2.PartitionCellByDatumPlane(datumPlane=d2[5],
		    cells=pickedCells)
		
		#============= Define Surfaces =================================
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Plate')
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate-dcort/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Top Cort')
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate-dcort-dtrab/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Trab')
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate-dcort-dtrab-(dcort-dxt)/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Bot Cort')
		
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Plate')
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate-dcort/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Top Cort')
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate-dcort-dtrab/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Trab')
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate-dcort-dtrab-(dcort-dxt)/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Bot Cort')
	
	###### Case 4 - Screw fully penetrates all bone layers	
	else:
		face1 = ScrewPart.faces.findAt((cx,cy,0.))
		ScrewPart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dcort)
		ScrewPart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dcort+dtrab)
		ScrewPart.DatumPlaneByOffset(plane=face1,flip=SIDE2, offset=dcort+dtrab+dcort)
		
		face2 = ScrewPart2.faces.findAt((cx2,cy2,0.))
		ScrewPart2.DatumPlaneByOffset(plane=face2,flip=SIDE2, offset=dcort)
		ScrewPart2.DatumPlaneByOffset(plane=face2,flip=SIDE2, offset=dcort+dtrab)
		ScrewPart2.DatumPlaneByOffset(plane=face2,flip=SIDE2, offset=dcort+dtrab+dcort)
		
		d1 = ScrewPart.datums
		pickedCells = ScrewPart.cells
		ScrewPart.PartitionCellByDatumPlane(datumPlane=d1[3],
		    cells=pickedCells)
		pickedCells = ScrewPart.cells
		ScrewPart.PartitionCellByDatumPlane(datumPlane=d1[4],
		    cells=pickedCells)
		pickedCells = ScrewPart.cells
		ScrewPart.PartitionCellByDatumPlane(datumPlane=d1[5],
		    cells=pickedCells)
		
		d2 = ScrewPart2.datums
		pickedCells = ScrewPart2.cells
		ScrewPart2.PartitionCellByDatumPlane(datumPlane=d2[3],
		    cells=pickedCells)
		pickedCells = ScrewPart2.cells
		ScrewPart2.PartitionCellByDatumPlane(datumPlane=d2[4],
		    cells=pickedCells)
		pickedCells = ScrewPart2.cells
		ScrewPart2.PartitionCellByDatumPlane(datumPlane=d2[5],
		    cells=pickedCells)
		
		#============= Define Surfaces =================================
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Plate')
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate-dcort/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Top Cort')
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate-dcort-dtrab/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Trab')
		face1 = ScrewPart.faces.findAt(((cx+R,cy,dscrew-dplate-dcort-dtrab-dcort/2),),)
		ScrewPart.Surface(side1Faces=face1, name='Bot Cort')
		
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Plate')
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate-dcort/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Top Cort')
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate-dcort-dtrab/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Trab')
		face1 = ScrewPart2.faces.findAt(((cx2+R,cy2,dscrew-dplate-dcort-dtrab-dcort/2),),)
		ScrewPart2.Surface(side1Faces=face1, name='Bot Cort')
		
	# ================= Partition Screw by Planes at 45 degrees ============
	ScrewPart.PartitionCellByPlaneThreePoints(
	    cells=ScrewPart.cells, point1=(cx,cy,0), point2=(cx+rp,cy+rp,0), point3=(cx+rp,cy+rp,dscrew))
	ScrewPart.PartitionCellByPlaneThreePoints(
	    cells=ScrewPart.cells, point1=(cx,cy,0), point2=(cx-rp,cy+rp,0), point3=(cx-rp,cy+rp,dscrew))
	
	ScrewPart2.PartitionCellByPlaneThreePoints(
	    cells=ScrewPart2.cells, point1=(cx2,cy2,0), point2=(cx2+rp,cy2+rp,0), point3=(cx2+rp,cy2+rp,dscrew))
	ScrewPart2.PartitionCellByPlaneThreePoints(
	    cells=ScrewPart2.cells, point1=(cx2,cy2,0), point2=(cx2-rp,cy2+rp,0), point3=(cx2-rp,cy2+rp,dscrew))

#*****************************************************************************
# Call Functions
#*****************************************************************************
createPartitionBone(myModel, myAssem)
createPartBone(myModel, myAssem)
createPartitionPlate(myModel, myAssem)
createPartPlate(myModel, myAssem)
createPartScrew(myModel, myAssem)

#*****************************************************************************
# Merge Solid Bone with Shell Partition
#*****************************************************************************
BoneInstance=myAssem.InstanceFromBooleanMerge(domain=
    GEOMETRY, instances=(
    myAssem.instances['Solid Bone'], 
    myAssem.instances['Bone Partition Part']), 
    keepIntersections=ON, name='Bone', originalInstances=SUPPRESS)
myAssem.features.changeKey(fromName='Bone-1', 
    toName='Bone')
BonePart = mdb.models['Bone and Screw'].parts['Bone']

#*****************************************************************************
# Merge Solid Plate with Shell Partition
#*****************************************************************************
PlateInstance=myAssem.InstanceFromBooleanMerge(domain=
    GEOMETRY, instances=(
    myAssem.instances['Solid Plate'], 
    myAssem.instances['Plate Partition Part']), 
    keepIntersections=ON, name='Plate', originalInstances=SUPPRESS)
myAssem.features.changeKey(fromName='Plate-1', 
    toName='Plate')
PlatePart = mdb.models['Bone and Screw'].parts['Plate']

#*****************************************************************************
# Create Screw Holes
#*****************************************************************************
ScrewHolesSketch=myModel.ConstrainedSketch(gridSpacing=0.36, name=
    'Screw Hole Sketch', sheetSize=14.96, transform=
    BonePart.MakeSketchTransform(
    sketchPlane=BonePart.faces.findAt((3.49/2,0,dbone)), 
    sketchPlaneSide=SIDE1, 
    sketchUpEdge=BonePart.edges.findAt((14.96,0,dbone)), 
    sketchOrientation=RIGHT, origin=(0, 0, dbone)))
BonePart.projectReferencesOntoSketch(filter=
    COPLANAR_EDGES, sketch=ScrewHolesSketch)

rp= R*cos(pi/4)
ScrewHolesSketch.CircleByCenterPerimeter(center=(cx,cy), point1=(cx+rp,cy+rp))
ScrewHolesSketch.CircleByCenterPerimeter(center=(cx2,cy2), point1=(cx2+rp,cy2+rp))

ScrewHolesSketch2=myModel.ConstrainedSketch(gridSpacing=0.36, name=
    'Screw Hole Sketch 2', sheetSize=14.96, transform=
    PlatePart.MakeSketchTransform(
    sketchPlane=PlatePart.faces.findAt((2.46/2,0,dbone+dplate)), 
    sketchPlaneSide=SIDE1, 
    sketchUpEdge=PlatePart.edges.findAt((7.94,0,dbone+dplate)), 
    sketchOrientation=RIGHT, origin=(0, 0, dbone+dplate)))
PlatePart.projectReferencesOntoSketch(filter=
    COPLANAR_EDGES, sketch=ScrewHolesSketch2)

rp= R*cos(pi/4)
ScrewHolesSketch2.CircleByCenterPerimeter(center=(cx,cy), point1=(cx+rp,cy+rp))
ScrewHolesSketch2.CircleByCenterPerimeter(center=(cx2,cy2), point1=(cx2+rp,cy2+rp))

BonePart.CutExtrude(depth=dscrew-dplate, 
    flipExtrudeDirection=OFF, sketch=
    ScrewHolesSketch, sketchOrientation=
    RIGHT, sketchPlane=BonePart.faces.findAt((3.49/2,0,dbone)), 
    sketchPlaneSide=SIDE1, sketchUpEdge=
    BonePart.edges.findAt((14.96,0,dbone)))

PlatePart.CutExtrude(depth=dplate, 
    flipExtrudeDirection=OFF, sketch=
    ScrewHolesSketch2, sketchOrientation=
    RIGHT, sketchPlane=PlatePart.faces.findAt((2.46/2,0,dbone+dplate)), 
    sketchPlaneSide=SIDE1, sketchUpEdge=
    PlatePart.edges.findAt((7.94,0,dbone+dplate)))

#*****************************************************************************
# Mesh Parts
#*****************************************************************************
BonePart.seedPart(deviationFactor=0.1, size=meshSize)
BonePart.generateMesh()

PlatePart.seedPart(deviationFactor=0.1, size=meshSize)
PlatePart.generateMesh()

ScrewPart = mdb.models['Bone and Screw'].parts['Screw 1']
ScrewPart2 = mdb.models['Bone and Screw'].parts['Screw 2']

ScrewPart.seedPart(deviationFactor=0.1, size=meshSize)
ScrewPart.generateMesh()
ScrewPart2.seedPart(deviationFactor=0.1, size=meshSize)
ScrewPart2.generateMesh()

#*****************************************************************************
#Create Surfaces
#*****************************************************************************

face1 = PlatePart.faces.findAt(((cx+R,cy,dbone+dplate/2),),
	((cx,cy+R,dbone+dplate/2),), ((cx-R,cy,dbone+dplate/2),),
	((cx,cy-R,dbone+dplate/2),),)
PlatePart.Surface(side1Faces=face1, name='Int 1')
face1 = PlatePart.faces.findAt(((cx2+R,cy2,dbone+dplate/2),),
	((cx2,cy2+R,dbone+dplate/2),), ((cx2-R,cy2,dbone+dplate/2),),
	((cx2,cy2-R,dbone+dplate/2),),)
PlatePart.Surface(side1Faces=face1, name='Int 2')

###### Case 1 - Screw only partially penetrates trabecular
if dscrew<(dplate+dcort+dtrab):
	dtrabpen = dscrew-dplate-dcort
	face1 = BonePart.faces.findAt(((cx+R,cy,dbone-dcort/2),),
		((cx,cy+R,dbone-dcort/2),), ((cx-R,cy,dbone-dcort/2),),
		((cx,cy-R,dbone-dcort/2),),)
	BonePart.Surface(side1Faces=face1, name='Top Cort')
	face1 = BonePart.faces.findAt(((cx+R,cy,dbone-dcort-dtrabpen/2),),
		((cx,cy+R,dbone-dcort-dtrabpen/2),), ((cx-R,cy,dbone-dcort-dtrabpen/2),),
		((cx,cy-R,dbone-dcort-dtrabpen/2),),)
	BonePart.Surface(side1Faces=face1, name='Trab')
	face1 = BonePart.faces.findAt(((cx2+R,cy2,dbone-dcort/2),),
		((cx2,cy2+R,dbone-dcort/2),), ((cx2-R,cy2,dbone-dcort/2),),
		((cx2,cy2-R,dbone-dcort/2),),)
	BonePart.Surface(side1Faces=face1, name='Top Cort 2')
	face1 = BonePart.faces.findAt(((cx2+R,cy2,dbone-dcort-dtrabpen/2),),
		((cx2,cy2+R,dbone-dcort-dtrabpen/2),), ((cx2-R,cy2,dbone-dcort-dtrabpen/2),),
		((cx2,cy2-R,dbone-dcort-dtrabpen/2),),)
	BonePart.Surface(side1Faces=face1, name='Trab 2')
		
	myAssem.SurfaceByMerge(name='Hole Interior', 
	    surfaces=(
	    myAssem.instances['Bone'].surfaces['Top Cort'], 
	    myAssem.instances['Bone'].surfaces['Trab']))
	myAssem.SurfaceByMerge(name='Hole 2 Interior', 
	    surfaces=(
	    myAssem.instances['Bone'].surfaces['Top Cort 2'], 
	    myAssem.instances['Bone'].surfaces['Trab 2']))
	myAssem.SurfaceByMerge(name='Screw 1 Bone Contact Area', 
	    surfaces=(
	    myAssem.instances['Screw 1'].surfaces['Top Cort'], 
	    myAssem.instances['Screw 1'].surfaces['Trab']))
	myAssem.SurfaceByMerge(name='Screw 2 Bone Contact Area', 
	    surfaces=(
	    myAssem.instances['Screw 2'].surfaces['Top Cort'], 
	    myAssem.instances['Screw 2'].surfaces['Trab']))
	
	

###### Case 2 - Screw fully penetrates trabecular but doesn't penetrate bottom cortical
elif dscrew==(dplate+dcort+dtrab):
	face1 = BonePart.faces.findAt(((cx+R,cy,dbone-dcort/2),),
		((cx,cy+R,dbone-dcort/2),), ((cx-R,cy,dbone-dcort/2),),
		((cx,cy-R,dbone-dcort/2),),)
	BonePart.Surface(side1Faces=face1, name='Top Cort')
	face1 = BonePart.faces.findAt(((cx+R,cy,dbone-dcort-dtrab/2),),
		((cx,cy+R,dbone-dcort-dtrab/2),), ((cx-R,cy,dbone-dcort-dtrab/2),),
		((cx,cy-R,dbone-dcort-dtrab/2),),)
	BonePart.Surface(side1Faces=face1, name='Trab')
	face1 = BonePart.faces.findAt(((cx2+R,cy2,dbone-dcort/2),),
		((cx2,cy2+R,dbone-dcort/2),), ((cx2-R,cy2,dbone-dcort/2),),
		((cx2,cy2-R,dbone-dcort/2),),)
	BonePart.Surface(side1Faces=face1, name='Top Cort 2')
	face1 = BonePart.faces.findAt(((cx2+R,cy2,dbone-dcort-dtrab/2),),
		((cx2,cy2+R,dbone-dcort-dtrab/2),), ((cx2-R,cy2,dbone-dcort-dtrab/2),),
		((cx2,cy2-R,dbone-dcort-dtrab/2),),)
	BonePart.Surface(side1Faces=face1, name='Trab 2')
	
	myAssem.SurfaceByMerge(name='Hole Interior', 
	    surfaces=(
	    myAssem.instances['Bone'].surfaces['Top Cort'], 
	    myAssem.instances['Bone'].surfaces['Trab']))
	myAssem.SurfaceByMerge(name='Hole 2 Interior', 
	    surfaces=(
	    myAssem.instances['Bone'].surfaces['Top Cort 2'], 
	    myAssem.instances['Bone'].surfaces['Trab 2']))
	myAssem.SurfaceByMerge(name='Screw 1 Bone Contact Area', 
	    surfaces=(
	    myAssem.instances['Screw 1'].surfaces['Top Cort'], 
	    myAssem.instances['Screw 1'].surfaces['Trab']))
	myAssem.SurfaceByMerge(name='Screw 2 Bone Contact Area', 
	    surfaces=(
	    myAssem.instances['Screw 2'].surfaces['Top Cort'], 
	    myAssem.instances['Screw 2'].surfaces['Trab']))

###### Case 3 - Screw fully penetrates trabecular and partially penetrates bottom cortical	
elif dscrew>(dplate+dcort+dtrab) and dscrew<(dplate+dbone):
	dxt = dplate+dbone-dscrew
	
	face1 = BonePart.faces.findAt(((cx+R,cy,dbone-dcort/2),),
		((cx,cy+R,dbone-dcort/2),), ((cx-R,cy,dbone-dcort/2),),
		((cx,cy-R,dbone-dcort/2),),)
	BonePart.Surface(side1Faces=face1, name='Top Cort')
	face1 = BonePart.faces.findAt(((cx+R,cy,dbone-dcort-dtrab/2),),
		((cx,cy+R,dbone-dcort-dtrab/2),), ((cx-R,cy,dbone-dcort-dtrab/2),),
		((cx,cy-R,dbone-dcort-dtrab/2),),)
	BonePart.Surface(side1Faces=face1, name='Trab')
	face1 = BonePart.faces.findAt(((cx2+R,cy2,dbone-dcort/2),),
		((cx2,cy2+R,dbone-dcort/2),), ((cx2-R,cy2,dbone-dcort/2),),
		((cx2,cy2-R,dbone-dcort/2),),)
	BonePart.Surface(side1Faces=face1, name='Top Cort 2')
	face1 = BonePart.faces.findAt(((cx2+R,cy2,dbone-dcort-dtrab/2),),
		((cx2,cy2+R,dbone-dcort-dtrab/2),), ((cx2-R,cy2,dbone-dcort-dtrab/2),),
		((cx2,cy2-R,dbone-dcort-dtrab/2),),)
	BonePart.Surface(side1Faces=face1, name='Trab 2')
	face1 = BonePart.faces.findAt(((cx+R,cy,dbone-dcort-dtrab-(dcort-dxt)/2),),
		((cx,cy+R,dbone-dcort-dtrab-(dcort-dxt)/2),), ((cx-R,cy,dbone-dcort-dtrab-(dcort-dxt)/2),),
		((cx,cy-R,dbone-dcort-dtrab-(dcort-dxt)/2),),)
	BonePart.Surface(side1Faces=face1, name='Bot Cort')
	face1 = BonePart.faces.findAt(((cx2+R,cy2,dbone-dcort-dtrab-(dcort-dxt)/2),),
	((cx2,cy2+R,dbone-dcort-dtrab-(dcort-dxt)/2),), ((cx2-R,cy2,dbone-dcort-dtrab-(dcort-dxt)/2),),
	((cx2,cy2-R,dbone-dcort-dtrab-(dcort-dxt)/2),),)
	BonePart.Surface(side1Faces=face1, name='Bot Cort 2')
	
	myAssem.SurfaceByMerge(name='Hole Interior', 
	    surfaces=(
	    myAssem.instances['Bone'].surfaces['Top Cort'], 
	    myAssem.instances['Bone'].surfaces['Trab'],
	    myAssem.instances['Bone'].surfaces['Bot Cort']))
	myAssem.SurfaceByMerge(name='Hole 2 Interior', 
	    surfaces=(
	    myAssem.instances['Bone'].surfaces['Top Cort 2'], 
	    myAssem.instances['Bone'].surfaces['Trab 2'],
	    myAssem.instances['Bone'].surfaces['Bot Cort 2']))
	myAssem.SurfaceByMerge(name='Screw 1 Bone Contact Area', 
	    surfaces=(
	    myAssem.instances['Screw 1'].surfaces['Top Cort'], 
	    myAssem.instances['Screw 1'].surfaces['Trab'],
	    myAssem.instances['Screw 1'].surfaces['Bot Cort']))
	myAssem.SurfaceByMerge(name='Screw 2 Bone Contact Area', 
	    surfaces=(
	    myAssem.instances['Screw 2'].surfaces['Top Cort'], 
	    myAssem.instances['Screw 2'].surfaces['Trab'],
	    myAssem.instances['Screw 2'].surfaces['Bot Cort']))

###### Case 4 - Screw fully penetrates all bone layers	
else:
	face1 = BonePart.faces.findAt(((cx+R,cy,dbone-dcort/2),),
		((cx,cy+R,dbone-dcort/2),), ((cx-R,cy,dbone-dcort/2),),
		((cx,cy-R,dbone-dcort/2),),)
	BonePart.Surface(side1Faces=face1, name='Top Cort')
	face1 = BonePart.faces.findAt(((cx+R,cy,dbone-dcort-dtrab/2),),
		((cx,cy+R,dbone-dcort-dtrab/2),), ((cx-R,cy,dbone-dcort-dtrab/2),),
		((cx,cy-R,dbone-dcort-dtrab/2),),)
	BonePart.Surface(side1Faces=face1, name='Trab')
	face1 = BonePart.faces.findAt(((cx2+R,cy2,dbone-dcort/2),),
		((cx2,cy2+R,dbone-dcort/2),), ((cx2-R,cy2,dbone-dcort/2),),
		((cx2,cy2-R,dbone-dcort/2),),)
	BonePart.Surface(side1Faces=face1, name='Top Cort 2')
	face1 = BonePart.faces.findAt(((cx2+R,cy2,dbone-dcort-dtrab/2),),
		((cx2,cy2+R,dbone-dcort-dtrab/2),), ((cx2-R,cy2,dbone-dcort-dtrab/2),),
		((cx2,cy2-R,dbone-dcort-dtrab/2),),)
	BonePart.Surface(side1Faces=face1, name='Trab 2')
	face1 = BonePart.faces.findAt(((cx+R,cy,dbone-dcort-dtrab-dcort/2),),
		((cx,cy+R,dbone-dcort-dtrab-dcort/2),), ((cx-R,cy,dbone-dcort-dtrab-dcort/2),),
		((cx,cy-R,dbone-dcort-dtrab-dcort/2),),)
	BonePart.Surface(side1Faces=face1, name='Bot Cort')
	face1 = BonePart.faces.findAt(((cx2+R,cy2,dbone-dcort-dtrab-dcort/2),),
	((cx2,cy2+R,dbone-dcort-dtrab-dcort/2),), ((cx2-R,cy2,dbone-dcort-dtrab-dcort/2),),
	((cx2,cy2-R,dbone-dcort-dtrab-dcort/2),),)
	BonePart.Surface(side1Faces=face1, name='Bot Cort 2')
	
	myAssem.SurfaceByMerge(name='Hole Interior', 
	    surfaces=(
	    myAssem.instances['Bone'].surfaces['Top Cort'], 
	    myAssem.instances['Bone'].surfaces['Trab'],
	    myAssem.instances['Bone'].surfaces['Bot Cort']))
	myAssem.SurfaceByMerge(name='Hole 2 Interior', 
	    surfaces=(
	    myAssem.instances['Bone'].surfaces['Top Cort 2'], 
	    myAssem.instances['Bone'].surfaces['Trab 2'],
	    myAssem.instances['Bone'].surfaces['Bot Cort 2']))
	myAssem.SurfaceByMerge(name='Screw 1 Bone Contact Area', 
	    surfaces=(
	    myAssem.instances['Screw 1'].surfaces['Top Cort'], 
	    myAssem.instances['Screw 1'].surfaces['Trab'],
	    myAssem.instances['Screw 1'].surfaces['Bot Cort']))
	myAssem.SurfaceByMerge(name='Screw 2 Bone Contact Area', 
	    surfaces=(
	    myAssem.instances['Screw 2'].surfaces['Top Cort'], 
	    myAssem.instances['Screw 2'].surfaces['Trab'],
	    myAssem.instances['Screw 1'].surfaces['Bot Cort']))
	

#*****************************************************************************
#Create Sets
#*****************************************************************************
face1 = BoneInstance.faces.findAt(((1.745,-5.715,dcort/2),),
	((1.745,-5.715,dcort+dtrab/2),), ((1.745,-5.715,dcort+dtrab+dcort/2),),
	((5.715,-5.715,dcort/2),), ((5.715,-5.715,dcort+dtrab/2),),
	((5.715,-5.715,dcort+dtrab+dcort/2),), ((11.45,-5.715,dcort/2),),
	((11.45,-5.715,dcort+dtrab/2),), ((11.45,-5.715,dcort+dtrab+dcort/2),),)
myAssem.Set(faces=face1, name='Bone -Y Plane')
face1 = BoneInstance.faces.findAt(((14.96,-3.97,dcort/2),),
	((14.96,-3.97,dcort+dtrab/2),), ((14.96,-3.97,dcort+dtrab+dcort/2),),
	((14.96,0,dcort/2),), ((14.96,0,dcort+dtrab/2),),
	((14.96,0,dcort+dtrab+dcort/2),), ((14.96,3.52,dcort/2),),
	((14.96,3.52,dcort+dtrab/2),), ((14.96,3.52,dcort+dtrab+dcort/2),),
	((14.96,7.04,dcort/2),),
	((14.96,7.04,dcort+dtrab/2),), ((14.96,7.04,dcort+dtrab+dcort/2),),)
myAssem.Set(faces=face1, name='Bone X Plane')

face1 = PlateInstance.faces.findAt(((0.,0.,dbone+dplate),),)
myAssem.Set(faces=face1, name='Plate -X Plane')
face1 = PlateInstance.faces.findAt(((5.715,-5.715,dbone+dplate),),)
myAssem.Set(faces=face1, name='Plate -Y Plane')

# *****************************************************************************
# Define Contact
# *****************************************************************************                    
myModel.ContactProperty('Lagrange Friction')
myModel.interactionProperties['Lagrange Friction'].TangentialBehavior(
    dependencies=0, directionality=ISOTROPIC, formulation=LAGRANGE, 
    pressureDependency=OFF, shearStressLimit=None, slipRateDependency=OFF, 
    table=((fricFact, ), ), temperatureDependency=OFF)

myModel.ContactProperty('Rough Contact')
myModel.interactionProperties['Rough Contact'].TangentialBehavior(
    formulation=ROUGH)

mdb.models['Bone and Screw'].ContactProperty(
    'Coulomb Friction (Penalty)')
mdb.models['Bone and Screw'].interactionProperties['Coulomb Friction (Penalty)'].TangentialBehavior(
    dependencies=0, directionality=ISOTROPIC, elasticSlipStiffness=None, 
    formulation=PENALTY, fraction=0.005, maximumElasticSlip=FRACTION, 
    pressureDependency=OFF, shearStressLimit=None, slipRateDependency=OFF, 
    table=((fricFact, ), ), temperatureDependency=OFF)


# ================= Tie Constraints ===========================================
myModel.Tie(adjust=ON, master=
    myAssem.instances['Plate'].surfaces['Int 1']
    , name='Tie Screw 1 to Plate', positionToleranceMethod=COMPUTED, slave=
    myAssem.instances['Screw 1'].surfaces['Plate']
    , thickness=ON, tieRotations=ON)
myModel.Tie(adjust=ON, master=
    myAssem.instances['Plate'].surfaces['Int 2']
    , name='Tie Screw 2 to Plate', positionToleranceMethod=COMPUTED, slave=
    myAssem.instances['Screw 2'].surfaces['Plate']
    , thickness=ON, tieRotations=ON)

# ================= Interactions ==============================================

myModel.SurfaceToSurfaceContactStd(adjustMethod=NONE, 
    clearanceRegion=None, createStepName='Initial', datumAxis=None, 
    initialClearance=OMIT, interactionProperty='Lagrange Friction', master=
    myAssem.surfaces['Screw 1 Bone Contact Area']
    , name='Screw 1 and Bone', slave=
    myAssem.surfaces['Hole Interior']
    , sliding=SMALL, thickness=ON)
myModel.SurfaceToSurfaceContactStd(adjustMethod=NONE, 
    clearanceRegion=None, createStepName='Initial', datumAxis=None, 
    initialClearance=OMIT, interactionProperty='Lagrange Friction', master=
    myAssem.surfaces['Screw 2 Bone Contact Area']
    , name='Screw 2 and Bone', slave=
    myAssem.surfaces['Hole 2 Interior']
    , sliding=SMALL, thickness=ON)

if contactForm=='Rough':
	myModel.interactions['Screw 1 and Bone'].setValuesInStep(
	    interactionProperty='Rough Contact', stepName='Initial')
	myModel.interactions['Screw 2 and Bone'].setValuesInStep(
	    interactionProperty='Rough Contact', stepName='Initial')
	
if contactForm=='Coulomb':
	myModel.interactions['Screw 1 and Bone'].setValuesInStep(
	    interactionProperty='Coulomb Friction (Penalty)', stepName='Initial')
	myModel.interactions['Screw 2 and Bone'].setValuesInStep(
	    interactionProperty='Coulomb Friction (Penalty)', stepName='Initial')

# *****************************************************************************
# Loads
# *****************************************************************************
myModel.StaticStep(name='Loads (Static, General)', 
    previous='Initial')
myModel.steps['Loads (Static, General)'].setValues(
    initialInc=0.1, maxInc=0.1)

region = myAssem.sets['Bone X Plane']
myModel.DisplacementBC(amplitude=UNSET, 
    createStepName='Loads (Static, General)', distributionType=UNIFORM, 
    fieldName='', fixed=OFF, localCsys=None, name='Disp Load of Bone X Plane', 
    region=region, 
    u1=DispLoad, u2=UNSET, u3=UNSET, ur1=UNSET, ur2=UNSET, ur3=UNSET)

# *****************************************************************************
# Boundary Conditions
# *****************************************************************************
region = myAssem.sets['Bone X Plane']
myModel.DisplacementBC(amplitude=UNSET, createStepName=
	    'Initial', distributionType=UNIFORM, fieldName='', localCsys=None, name=
	    'Fix Z Disp of Bone X Plane', region=region, u1=UNSET, u2=UNSET,
	    u3=SET, ur1=UNSET, ur2=UNSET, ur3=
	    UNSET)
region = myAssem.sets['Bone -Y Plane']
myModel.YsymmBC(createStepName='Initial', name='Y Symm of Bone -Y Plane', 
	    region=region)
region = myAssem.sets['Plate -Y Plane']
myModel.YsymmBC(createStepName='Initial', name='Y Symm of Plate -Y Plane', 
	    region=region)
region = myAssem.sets['Plate -X Plane']
myModel.XsymmBC(createStepName='Initial', name='X Symm of Plate -X Plane', 
    region=region)

# *****************************************************************************
# Delete Extra Parts Used in Construction
# *****************************************************************************
del myModel.parts['Bone Partition']
del myModel.parts['Plate Partition']
del myModel.parts['Solid Bone']
del myModel.parts['Solid Plate']

myAssem.deleteFeatures(('Bone Partition Part','Plate Partition Part',
			'Solid Bone','Solid Plate'))

# *****************************************************************************
# Create Job
# *****************************************************************************
mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, 
    explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF, 
    memory=90, memoryUnits=PERCENTAGE, model='Bone and Screw', modelPrint=
    OFF, name='Job-1', nodalOutputPrecision=SINGLE, queue=None, scratch='', 
    type=ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)
mdb.jobs['Job-1'].setValues(numCpus=2, numDomains=2)

