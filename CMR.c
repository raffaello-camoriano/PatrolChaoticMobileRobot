//*******************************************************************
// Chaotic Mobile Robot for Patrol Application in Hostile Environment
//*******************************************************************


//Please compile using: gcc -pthread CMR.c -lm -lrt -lSDL

#include <math.h>
#include <time.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include "SDL/SDL.h"

#define N 5				// number of first order equations 
#define MAX 50000.0		// simulation time in sec.
#define PI 3.14159265
#define M 10            // No. of agents (CMR+HA)
#define XMAX 10   		// X boundary of Patrol Area
#define YMAX 10  		// Y boundary of Patrol Area  
#define RANGE 0.5 		// Combat engagement range

// Global Variables

double dist = 0.01;		// Stepsize in t
double X[N][M];  		// M agents, N states each
short alive[M]; 		// Bit-vector which records if the corresponding agent is in alive, ghost or return mode
short GUI_state = 0;	// Shared variable used to control the GUI thread
short GUI_result;		// Result of the simulation, to be passed to the GUI

// Synchronization tools

pthread_barrier_t check_combat_barrier;		//It is used to synchronize the combat check routine in the main program with the agent threads
pthread_barrier_t begin_rendering_barrier;	//Create a barrier to start the rendering of the positions of the agents, performed by the GUI thread

pthread_mutex_t X_mutex = PTHREAD_MUTEX_INITIALIZER;			//Create mutex to protect the X variable, which is accessed by the threads
pthread_mutex_t alive_mutex = PTHREAD_MUTEX_INITIALIZER;		//Create mutex to protect the alive variable, which is accessed by the threads
pthread_mutex_t GUI_mutex = PTHREAD_MUTEX_INITIALIZER;			//Create mutex to protect the GUI state
pthread_mutex_t GUI_result_mutex = PTHREAD_MUTEX_INITIALIZER;	//Create mutex to protect the result of the simulation passet to the GUI

#include "functions.h"
#include "agent.h"
#include "GUI.h"

//Main program

int main( int argc, char* args[] )	{

	//Launch GUI thread
	
	pthread_barrier_init(&begin_rendering_barrier , NULL , 2 );		//GUI sync barrier initialization

	pthread_t GUI;		
	pthread_create( &GUI, NULL, &GUI_fun, NULL);	//Initialize GUI thread
	printf("GUI thread initialized.\n");

  	// Initalizing X[][] at the beginning

	int i,j;
	
	//Initialize random seed
	
	time_t seconds;
	time(&seconds);
	srand((unsigned int) seconds);
        
	for (i=0; i< M; ++i) {      // The first thread should be the robot
 		
		if ( i == 0 ) {
			X[0][i]= RND()+ 1.0; 
			X[1][i]= RND()+ 1.0;
			X[2][i]= X[2][i]= 20*PI*RND();				//Randomized initial orientation
			X[3][i]= RND()+ 1;	//Initial x
			X[4][i]= RND()+ 1;	//Initial y
		}
		
		else	{
			X[0][i]= RND()+ 1.0; 
			X[1][i]= RND()+ 1.0;
			X[2][i]= X[2][i]= 20*PI*RND();				//Randomized initial orientation
			X[3][i]= RND()+ 7;	//Initial x
			X[4][i]= RND()+ 7;	//Initial y
		}

		alive[i] = 1;	// All the agents are alive at the beginning	
	}

	pthread_t AG[M];  	// Agent threads declaration
		
	//Barrier initialization
	
	pthread_barrier_init(&check_combat_barrier , NULL , M+1 );		//M threads + proximity check routine
	
	// Nanosleep for 5 seconds for letting the GUI show the welcome message
    struct timespec req;
    req.tv_sec=5;
    req.tv_nsec=0;
    while(nanosleep(&req,&req)==-1)
         continue;

	// Begin rendering
	
	pthread_mutex_lock (&GUI_mutex);
	GUI_state = 1;
	pthread_mutex_unlock (&GUI_mutex);
	pthread_barrier_wait (&begin_rendering_barrier);

	printf("Cyclic timer created and initialized.\n");
	
	// Creating a thread for each agent
	
 	for( i=0 ; i < M ; ++i ) {
		   
		pthread_create( &AG[i], NULL, &agent, (void*) i);
	}

	printf("Threads created.\n");

	//*** Routine to catch hostile agents and kill them
	
	printf("\n\n**** Proximity routine started ****\n\n");

	//Proximity check useful variables

   	int HA = M-1; 			// No. of hostile agents
   	int enemies_count; 		// Number of hostile agents found inside the combat range
   	int single_enemy_id; 	// If there is a single enemy in the combat range, its id is stored here
   	short stop_flag = 0;
	float distance = 0;		// Distance between the agent and the mobile robot
	
	do {	

		pthread_barrier_wait (&check_combat_barrier);	//Wait for the new positions to be computed by the threads at each step

		enemies_count = 0;

		for ( i=1 ; i < M ; ++i ) {

			pthread_mutex_lock (&alive_mutex);
			if ( alive[i] == 1 )	{	// Consider the thread only if it is alive
			pthread_mutex_unlock (&alive_mutex);
			
				//Check how many hostile agents are there in the combat range of the mobile robot

				pthread_mutex_lock (&X_mutex);
				distance = sqrt(pow(X[3][i]-X[3][0],2) + pow(X[4][i]-X[4][0],2));
				pthread_mutex_unlock (&X_mutex);

				// Check on a circular proximity area
				if ( distance <= RANGE ) {
			
					++enemies_count;
					single_enemy_id = i;	//Store the targeted hostile agent's ID
				}
			}
			else	{
				pthread_mutex_unlock (&alive_mutex);
			}
		}
			
		if (enemies_count == 1) {  // 1 vs.1: the agent thread loses and is transformed into a ghost

			--HA;	//Decrease the number of alive hostile agents

			pthread_mutex_lock (&alive_mutex);
			alive[single_enemy_id] = 0;		//The hostile agent's mode is set to "ghost"
			pthread_mutex_unlock (&alive_mutex);
			
			printf("Hostile Agent %d Killed!\n", single_enemy_id);
			
			if (HA > 1) {
				printf("There are still %d HAs\n", HA);				
			}
			else if (HA == 1)	{
				printf("Only %d hostile agent left!\n", HA);				
			}
		}
		
		if ( (enemies_count > 1) || (HA == 0) ) {
		
			pthread_mutex_lock (&alive_mutex);
			for ( j = 0; j<M ; ++j)	{
				alive[j] = 2;	// The threads are set to return from execution
			}
			pthread_mutex_unlock (&alive_mutex);

			if (enemies_count > 1)	{
				printf("Mobile robot killed by %d hostile agents, game over.\n", enemies_count);
				pthread_mutex_lock (&GUI_result_mutex);
				GUI_result = 0;	//Result to be displayed by the GUI: game over
				pthread_mutex_unlock (&GUI_result_mutex);
			}
			else	{
				printf("All the hostile agents have been catched by the mobile robot.\n");
				pthread_mutex_lock (&GUI_result_mutex);
				GUI_result = 1;	//Result to be displayed by the GUI: game won
				pthread_mutex_unlock (&GUI_result_mutex);
			}
			stop_flag = 1;
		}
		
		pthread_barrier_wait (&check_combat_barrier);	//The threads must wait for the routine to complete
		
	}	while ( stop_flag != 1); // until all hostile agents are killed


	for (i = 0 ; i<M; ++i)	{
		pthread_join(AG[i], NULL);	//Join agent threads
	}
	
	//Join GUI thread
	
	pthread_mutex_lock (&GUI_mutex);
	GUI_state = 2;
	pthread_mutex_unlock (&GUI_mutex);
	
	pthread_join(GUI, NULL);
	
	//Destroy barriers
	pthread_barrier_destroy (&check_combat_barrier);
	pthread_barrier_destroy (&begin_rendering_barrier);
	printf("Barriers deleted!\n");
	
	//Destroy mutexes
	pthread_mutex_destroy (&GUI_mutex);
	pthread_mutex_destroy (&GUI_result_mutex);
	pthread_mutex_destroy (&X_mutex);
	pthread_mutex_destroy (&alive_mutex);
	printf("Mutexes deleted!\n");

	printf("Main program ends!\n");

	return 0;
}
