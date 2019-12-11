// Each agent will receive initial condition and will run independently in the workarea chaotically

void *agent (void *p){
	
	int ID;
        ID = (int)p;    	 // Agent ID will specify which portion of X we should work on
	int i;       		 	 // aux. counter
	long int j = 1;			 // Step counter
	long double t;      	 // elapsed time
	short mode;				 // Alive-Ghost-Terminate mode to be acquired

	double *y = (double*)malloc(N*sizeof(double));

	for( i=0 ; i < N ; ++i){
		y[i] = X[i][ID]; // aquiring initial condition
	}
	
	while (1) {			/* Execution loop */

		//Acquire execution mode at each step

		pthread_mutex_lock (&alive_mutex);
		mode = alive[ID];	
		pthread_mutex_unlock (&alive_mutex);

		//Mode switch

		switch (mode) {
		case 0:		// Ghost, simply passes the barriers

			pthread_barrier_wait (&check_combat_barrier);
			pthread_barrier_wait (&check_combat_barrier);	//The threads must wait for the routine to complete

			break;
		case 1:		// Alive, performs computation of next coordinates

	   		t = j * dist;           // delta t
	   		y = RK4(t, y, dist);   	// Runge-Kutta calculations, returns the new state vector

			++j;

			// Update the corresponding part in X

			pthread_mutex_lock (&X_mutex);
			for( i=0 ; i<N ; ++i ){
				X[i][ID] = y[i];  // states are updated after differentiation
			}
			pthread_mutex_unlock (&X_mutex);
			
			//Synchronize with the proximity routine
			pthread_barrier_wait (&check_combat_barrier);
			pthread_barrier_wait (&check_combat_barrier);	//The threads must wait for the routine to complete

			break;

		case 2:		//Terminate, the simulation is finished and the threads joined

			printf("Agent %d returns.\n", ID);
			pthread_exit(NULL);

		default:

			pthread_barrier_wait (&check_combat_barrier);   //The threads let the proximity routine start
			pthread_barrier_wait (&check_combat_barrier);	//The threads must wait for the routine to complete

			break;
		} 
				
	}
	return NULL;
}
