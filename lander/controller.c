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

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/* call this once to do one-time initialize: allocate memeory etc. */

void init_controller( SIM *s )
{
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
  int i;
  static int count = 0;
  double k_x = 1.0;
  double b_x = 2.0;
  double k_r = 1.0;
  double b_r = 2.0;
  double q_minus[N_Q];
  double q_diff[N_Q];

  count++;

  /*
  // Handy for generating a data file for Matlab
  if ( count % 1000 == 0 )
    {
      printf( "%d ", count );
      for( i = 0; i < s->n_markers; i++ )
	{
	  printf( "%20.15f %20.15f %20.15f ", s->markers_lander[i][0],
		  s->markers_lander[i][1], s->markers_lander[i][2] );
	}
      printf( "\n" );
    }
  */

//convert markers_lander (markers in lander frame) to world frame by 
//subtracting lander location from marker coordinates 
double markers_world_calc[s->n_markers][N_XYZ];
for (i=0; i<s->n_markers; i++)
{
subtract_v3(&(s->markers_lander[i][0]), s->lander_x, *markers_world_calc);
	int j;
	if (count % 1000 ==0)
	{	
		for (j=0; j<N_XYZ; j++)
		{
		//printf(" %lf ", markers_world_calc[i][j]);
		}
		//printf("\n");
	}	
}

// read in file of predicted alien object COM locations. I'm sorry.
  FILE *file;
  file = fopen("com_traj.txt", "r");
  i = 0;
  int j = 0;
  float com_traj[10001][4]; // array to store predicted COM values in, 1st column is time step, then x,y,z
  for(i=0;i<10001;i++)
  {
	for(j=0;j<4;j++)
	{
		fscanf(file, "%g", &com_traj[i][j]);
	}
   // this is just to make sure the file is actually getting read
//	if (i%1000 == 0)
//		printf("%g, %g, %g, %g\n", com_traj[i][0], com_traj[i][1], 			com_traj[i][2], com_traj[i][3]);
  
	
  }
  fclose(file);

int t_mid = 50; // point to switch from aiming at point to matching alien trajectory
double sf = 1.00; //scaling factor to account for everything being wrong


if (count <= t_mid)
{

  // desired lander position
  s->lander_x_d[XX] = com_traj[t_mid][XX+1]/sf + 5;
  s->lander_x_d[YY] = com_traj[t_mid][YY+1]/sf + 5;
  s->lander_x_d[ZZ] = com_traj[t_mid][ZZ+1]/sf + 5;

  // desired lander orientation

  s->lander_q_d[Q0] = com_traj[t_mid][4];
  s->lander_q_d[Q1] = com_traj[t_mid][5];
  s->lander_q_d[Q2] = com_traj[t_mid][6];
  s->lander_q_d[Q3] = com_traj[t_mid][7];
}
else
{

  // desired lander position

  s->lander_x_d[XX] = com_traj[count][XX+1]/sf;
  s->lander_x_d[YY] = com_traj[count][YY+1]/sf;
  s->lander_x_d[ZZ] = com_traj[count][ZZ+1]/sf;

  // desired lander orientation
//use these when we have the final quaternions...
  s->lander_q_d[Q0] = com_traj[count][4];
  s->lander_q_d[Q1] = com_traj[count][5];
  s->lander_q_d[Q2] = com_traj[count][6];
  s->lander_q_d[Q3] = com_traj[count][7];

 // given values
/*
  s->lander_q_d[Q0] = 0.70710678118655;
  s->lander_q_d[Q1] = 0.0;
  s->lander_q_d[Q2] = 0.0;
  s->lander_q_d[Q3] = 0.70710678118655;
*/


}
  // lander translational control (PD servo)
  for ( i = 0; i < 3; i++ )
    {
      s->lander_thrust_world[i] = 
	k_x*( s->lander_x_d[i] - s->lander_x[i] ) +
	b_x*( - s->lander_xd[i] );
    }
  multiply_transpose_m3_v3( s->lander_r, s->lander_thrust_world, 
			    s->lander_thrust );

  // lander orientation control
  // Attempt to do PD control, but orientations not vectors, so complicated
  // "subtract quaternions"
  invert_q( s->lander_q, q_minus );
  compose_q( q_minus, s->lander_q_d, q_diff );
  q_to_rotvec( q_diff, s->rotvec ); 
  // printf( "%g %g %g %g\n", q_diff[0], q_diff[1], q_diff[2], q_diff[3] );
  // printf( "%g %g %g\n", s->rotvec[0], s->rotvec[1], s->rotvec[2] );

  // PD servo for orientation
  // w and rotvec are in body coordinates.
  for ( i = 0; i < N_XYZ; i++ )
    {
      s->lander_torque[i] = k_r*s->rotvec[i] - b_r*s->lander_w[i];
    }
  multiply_m3_v3( s->lander_r, s->lander_torque, s->lander_torque_world );

  // To get better control should compute desired acceleration and use
  // inverse dynamics to compute torque.

  return 0;
}

/*****************************************************************************/
