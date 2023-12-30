using UnityEngine;
using System;
using System.Collections;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision

	Vector3[] vertices;

	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		// define the vertices parameters
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		// for each vertex, check if it is below the plane
		Vector3 avg_r = new Vector3(0, 0, 0); // relative to the center of mass
		int num_collision_points = 0;

		// compute the average Rr
		for (int i=0; i<vertices.Length; i++){
			Vector3 Rr_i = R.MultiplyPoint3x4(vertices[i]);
			Vector3 x_i = transform.position + Rr_i;
			if (Vector3.Dot(x_i - P, N) < 0){ // if the vertex is below the plane
				// check if the vert velocity is going into the plane
				Vector3 v_i = v + Vector3.Cross(w, Rr_i);
				if (Vector3.Dot(v_i, N) < 0){
					avg_r += vertices[i];
					num_collision_points += 1;
				}
			}
		}
		if (num_collision_points <= 0){
			return;
		}
		avg_r /= num_collision_points; // in local space

		// compute the wanted v_rebound_new
		Vector3 global_collision_point = transform.position + R.MultiplyPoint3x4(avg_r);
		Vector3 v_avg = v + Vector3.Cross(w, transform.rotation * avg_r);

		Vector3 v_avg_N = Vector3.Dot(v_avg, N) * N;
		Vector3 v_avg_T = v_avg - v_avg_N;
		Vector3 v_new_N = -restitution * v_avg_N;
		float alpha = Math.Max(0, 1 - restitution *(1 + restitution) * v_avg_N.magnitude / v_avg_T.magnitude);
		Vector3 v_new_T = alpha * v_avg_T;
		Vector3 v_new = v_new_N + v_new_T;

		// compute the impulse J
		Matrix4x4 Rr_star = Get_Cross_Matrix(R.MultiplyPoint3x4(avg_r));
		Matrix4x4 I = R * I_ref * R.transpose;
		Matrix4x4 I_inv = I.inverse;

		Matrix4x4 K = Matrix4x4.identity;
		float one_over_mass = 1.0f / mass;
		K[0, 0]*=one_over_mass;
		K[1, 1]*=one_over_mass;
		K[2, 2]*=one_over_mass;
		K[3, 3]*=one_over_mass;
		Matrix4x4 K_sub_target = Rr_star * I_inv * Rr_star;
		for (int i=0; i<4; i++){
			for (int j=0; j<4; j++){
				K[i, j] -= K_sub_target[i, j];
			}
		}
		Vector3 J = K.inverse * (v_new - v_avg);

		// update v and w
		v += J / mass;
		Vector4 w_delta = I_inv * Rr_star * J;
		w += new Vector3(w_delta.x, w_delta.y, w_delta.z);
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			w = new Vector3 (20f, 10f, 10f);
			launched=true;
		}

		// Part I: Update velocities
		if (launched) 
		{
			Vector3 v_mid = v + 0.5f * dt * (new Vector3 (0, -9.8f, 0));
			// Vector4 ang_acc = I_ref.inverse * Get_Cross_Matrix (w) * I_ref * w;
			// Vector3 w_mid = w + 0.5f * dt * (new Vector3 (ang_acc.x, ang_acc.y, ang_acc.z));

			Vector3 pos_new = transform.position + dt * v_mid;
			Quaternion rot_new = transform.rotation * Quaternion.Euler (dt * w);
			
			v = v_mid + 0.5f * dt * (new Vector3 (0, -9.8f, 0));
			// ang_acc = I_ref.inverse * Get_Cross_Matrix (w_mid) * I_ref * w_mid;
			// w = w_mid + 0.5f * dt * (new Vector3 (ang_acc.x, ang_acc.y, ang_acc.z));
			// add decay 
			v = v * linear_decay;
			w = w * angular_decay;

			transform.position = pos_new;
			transform.rotation = rot_new;
		}


		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x    = transform.position;
		//Update angular status
		Quaternion q = transform.rotation;

		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}
