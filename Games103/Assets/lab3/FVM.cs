using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] 		Tet;
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;
	Vector3[] 	V;
	Vector3[] 	X;
	int number;				//The number of vertices

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;
	int[]		V_num;

	SVD svd = new SVD();

	// add some custom parameters
	Vector3 prev_X;

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/lab3/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number*4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/lab3/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;
	    		float temp=X[i].y;
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

		//TODO: Need to allocate and assign inv_Dm
		inv_Dm = new Matrix4x4[tet_number];
		for(int t=0; t<tet_number; t++){
			inv_Dm[t] = Build_Dm(t);
		}
    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.
		int x0 = Tet[tet*4+0];
		int x1 = Tet[tet*4+1];
		int x2 = Tet[tet*4+2];
		int x3 = Tet[tet*4+3];
		Vector3 e1 = X[x1]-X[x0];
		Vector3 e2 = X[x2]-X[x0];
		Vector3 e3 = X[x3]-X[x0];
		ret.SetColumn(0, new Vector4(e1.x, e1.y, e1.z, 0));
		ret.SetColumn(1, new Vector4(e2.x, e2.y, e2.z, 0));
		ret.SetColumn(2, new Vector4(e3.x, e3.y, e3.z, 0));
		ret.SetColumn(3, new Vector4(0, 0, 0, 1));
		return ret;
	}

	Matrix4x4 Build_Dm(int tet)
	{
		return Build_Edge_Matrix(tet).inverse;
    }

	// matrix float multiplication
	Matrix4x4 Matrix_Float_Multiply(Matrix4x4 A, float b){
		Matrix4x4 ret = Matrix4x4.zero;
		for(int i=0; i<4; i++){
			for(int j=0; j<4; j++){
				ret[i, j] = A[i, j] * b;
			}
		}
		return ret;
	}

	// mattrix add
	Matrix4x4 Matrix_Add(Matrix4x4 A, Matrix4x4 B){
		Matrix4x4 ret = Matrix4x4.zero;
		for(int i=0; i<4; i++){
			for(int j=0; j<4; j++){
				ret[i, j] = A[i, j] + B[i, j];
			}
		}
		return ret;
	}

	// matrix trace
	float Matrix_Trace(Matrix4x4 A){
		float ret = 0;
		for(int i=0; i<4; i++){
			ret += A[i, i];
		}
		return ret;
	}


    void _Update()
    {
		// define some constants
		Vector3 gravity = new Vector3(0, -9.8f, 0);
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}

    	for(int i=0 ;i<number; i++)
    	{
    		//TODO: Add gravity to Force.
			Force[i]+=gravity*mass;
			// TODO: Add damping force to Force.
			V[i] *= damp;
    	}

    	for(int tet=0; tet<tet_number; tet++)
    	{
			int x0 = Tet[tet*4+0];
			int x1 = Tet[tet*4+1];
			int x2 = Tet[tet*4+2];
			int x3 = Tet[tet*4+3];
			Vector3 e1 = X[x1]-X[x0];
			Vector3 e2 = X[x2]-X[x0];
			Vector3 e3 = X[x3]-X[x0];
    		//TODO: Deformation Gradient
			Matrix4x4 F = Matrix4x4.zero;
				//  build current edge matrix
			Matrix4x4 E = Build_Edge_Matrix(tet);
				//  compute F
			F = E*inv_Dm[tet];
    		
    		//TODO: Green Strain = 0.5*(F^T*F-I)
			Matrix4x4 Green = Matrix4x4.zero;
			Green = F.transpose*F;
			Matrix4x4 I = Matrix4x4.identity;
				// compute Green Strain
			for(int i=0; i<4; i++){
				for(int j=0; j<4; j++){
					Green[i, j] -= I[i, j];
					Green[i, j] *= 0.5f;
				}
			}
			//TODO: Second PK Stress
			Matrix4x4 S = Matrix4x4.zero;
			S = Matrix_Add( Matrix_Float_Multiply(Green, 2 * stiffness_1), Matrix_Float_Multiply(Matrix4x4.identity, stiffness_0 * Matrix_Trace(Green)));
    		//TODO: Elastic Force
			float volume = Mathf.Abs(Vector3.Dot(e1, Vector3.Cross(e2, e3))) / 6.0f;
			Matrix4x4 vol_ref_FS = Matrix_Float_Multiply(F*S, volume);
			Vector3 f1 = vol_ref_FS.MultiplyPoint3x4(e1);
			Vector3 f2 = vol_ref_FS.MultiplyPoint3x4(e2);
			Vector3 f3 = vol_ref_FS.MultiplyPoint3x4(e3);
			Vector3 f0 = -(f1+f2+f3);
			Force[x0] += f0;
			Force[x1] += f1;
			Force[x2] += f2;
			Force[x3] += f3;
    	}

    	for(int i=0; i<number; i++)
    	{
    		//TODO: Update X and V here.

			// laplacian smoothing, storing the neighbors in a set, and then smoothing with the avg Force
			Vector3 a = Force[i] / mass;
			V[i] += a * dt;
			X[i] += V[i] * dt;

			// after updating X, we need to update the inv_Dm
			Force[i] = Vector3.zero;
    		//TODO: (Particle) collision with floor.
			if(X[i].y < -3){ // floor
				// add collision force
				Force[i] += new Vector3(0, 200, 0);
			}
    	}
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
