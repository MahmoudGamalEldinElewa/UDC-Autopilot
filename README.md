# UDC-Autopilot

This code solves 2-D Truss problems using the Stiffness method.
--------------------------------------------------------------
- The code should be able to deal with: 
 
i) Supports with known settlements. 
 
ii) Nodal loads. 
 
iii) Member loads. Any member may be subjected to any of the following member loads (or a combination of them) 
 
 Concentrated force at its midpoint.  Concentrated moment at its midpoint.  Uniform distributed load. 
 
- The code shows the following as outputs 
 
i) Nodal displacements. ii) Support Reactions. iii) Members end actions. iv) Deformed shape. v) Axial force, shear force and bending moment diagrams for all members.

----------------------------------------------------------------------------------------------------------------------------------------
The Main program consists of 3 parts:
- 
- Preprocessing:
Input Data (Member and Joint Information, Load, Constraints) 
• Build Model (Member stiffness matrix 𝑆𝑀 , Joint stiffness matrix 𝑆𝐽 ) 
• Combined joint load 𝐴𝐶 ( Fixed End Actions 𝐴𝑀𝐿 , Equivalent Load 𝐴𝐸 and Joint Load 𝐴 ) 
• Joint Displacement 𝐷𝐽

- Solution:
Rearrange Stiffness matrix and Partition 𝑆 , 𝑆𝐷𝑅 , 𝑆𝑅𝐷 , 𝑆𝑅𝑅 
• Rearrange Combined load and Partition ( 𝐴𝐷 , 𝐴𝑅𝐿 ) 
• Support settlement 𝐷𝑅 
• Calculate the unknown displacement 𝐷 
• Calculate the Reaction 𝐴𝑅

- Postprocessing:
Arbitrary Displacement vector 𝐷𝐽 
• Deformation plot 
• Member End Actions 𝐴𝑀 
• Forces and Moments Diagrams

----------------------------------------------------------------------------------------------------------------------------------------
This code is based on the stiffness method which can be found in 
Aircraft Structures for Engineering Students
Elsevier aerospace engineering series
Author	T.H.G. Megson
