/*****************************************************************************/
/*
  controller.c: control strategy.
*/
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "matrix3.h"
#include "main.h"
#include "main2.h"
#include "sdfast/alien.h"
#include "sdfast/lander.h"

#define N_INIT  1000000000
#define N_TRACK 500

double com_traj[10001][8]; // array to store predicted COM values in, 1st column is time step, then x,y,z
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/* call this once to do one-time initialize: allocate memeory etc. */

void init_controller( SIM *s )
{
// read in file of predicted alien object COM locations. I'm sorry.
  FILE *file;
  printf("Reading input file");
  file = fopen("com_traj.txt", "r");
  int i = 0;
  int j = 0;
  for(i=0;i<10001;i++)
  {
	for(j=0;j<8;j++)
	{
    float val;
    int ret =  fscanf(file, "%g", &val);
    com_traj[i][j] = val;
	}
   // this is just to make sure the file is actually getting read
//	if (i%1000 == 0)
//		printf("%g, %g, %g, %g\n", com_traj[i][0], com_traj[i][1], 			com_traj[i][2], com_traj[i][3]);
  
	
  }
  fclose(file);
}

/*****************************************************************************/
/* call this many times to restart a controller */

void reinit_controller( SIM *s )
{
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

int controller( SIM *s )
{

  static int count = 0;
  // Do nothing during initial monitoring period
  //if (count++ < N_INIT)
  //  return 0;

  int i, j;
  double k_x = 3.0;
  double b_x = 1.0;
  double k_r = 0.4;
  double b_r = 0.2;
  double q_minus[N_Q];
  double q_diff[N_Q];
  double alien_norm[3];
  double lander_norm[3];

  if (count < N_TRACK + N_INIT){
    
    // Convert orientaiton to rotation matrix
    double R[3][3];
    q_to_r(com_traj[N_TRACK]+4, R);

    // Compute required orientation
    double neg_body_norm[] = {-1, 0, 0};
    multiply_m3_v3(R, neg_body_norm, alien_norm);
  
    // Compute tracking position
    double body_track_pos[] = {5, 0, 0};
    double world_track_pos[3];
    multiply_m3_v3(R, world_track_pos, world_track_pos);
    for (i = 0; i < 3; i++)
      s->lander_x_d[i] = com_traj[count][i+1] + world_track_pos[i];    
  }
  else
  {
    //convert markers_lander (markers in lander frame) to world frame
    //by applying appropriate transformation:
    double markers_world[s->n_markers][N_XYZ];
    for (i=0; i<s->n_markers; i++)
    {
      double markers_temp[N_XYZ];
      double r_lander_world[3][3];
      q_to_r(s->lander_q, r_lander_world);    
      multiply_m3_v3(r_lander_world, s->markers_lander[i], markers_temp);     
      subtract_v3(markers_temp, s->lander_x, markers_world[i]);
    }

    // Compute face vectors
    double face7_5[N_XYZ], face5_4[N_XYZ];
    subtract_v3(markers_world[5], markers_world[7], face7_5);
    subtract_v3(markers_world[4], markers_world[5], face5_4);

    // Compute target normal
    cross_product_v3(face5_4, face7_5, alien_norm);

    // Compute target face centroid
    for (i = 0; i < 3; i++){
      s->lander_x_d[i] = 0;
      for (j = 4; j < 8; j++)
        s->lander_x_d[i] += markers_world[j][i];
      
      s->lander_x_d[i] /=  4.0;
    }
  } 

  // lander translational control (PD servo)
  for ( i = 0; i < 3; i++ )
  {
    s->lander_thrust_world[i] = 
      k_x*( s->lander_x_d[i] - s->lander_x[i] ) +
      b_x*( -s->lander_xd[i] );
  }
  multiply_transpose_m3_v3( s->lander_r, s->lander_thrust_world, 
                            s->lander_thrust );

  // Create difference quaternion from normals
  vec_diff_to_quat(alien_norm, lander_norm, q_diff); 
  q_to_rotvec( q_diff, s->rotvec ); 

  // PD servo for orientation
  // w and rotvec are in body coordinates.
  for ( i = 0; i < N_XYZ; i++ )
    s->lander_torque[i] = -k_r*s->rotvec[i] - b_r*(s->lander_w[i]);
  
  multiply_m3_v3( s->lander_r, s->lander_torque, s->lander_torque_world );
  if (count % 1000)
  {
    printf("an: %f %f %f ln: %f %f %f qd %f %f %f %f rv: %f %f %f\n", alien_norm[0], 
           alien_norm[1], alien_norm[2], lander_norm[0],
           lander_norm[1], lander_norm[2], q_diff[0], q_diff[1],
           q_diff[2], q_diff[3], s->rotvec[0], s->rotvec[1], s->rotvec[2]);
  }
  
  return 0;
}

/*****************************************************************************/
