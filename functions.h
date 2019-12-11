// Functions

/*
//*****************************************************************************
void RK4(double x, double y[], double step);	// Runge-Kutta function 
double DE(double x, double y[], int i);		// function for derivatives 
double  RND(void);                              // small random value function
//*****************************************************************************
*/

//Returns an array

double * DE(double x, double y[], double YDOT[]) {

	int j;

	double A=0.5; // system parameters
	double v=1;
	double B=0.25;
	double C=0.25;
	
	YDOT[0]=A* sin(y[2])+C* cos(y[1]);
	YDOT[1]=B* sin(y[0])+A* cos(y[2]);
	YDOT[2]=C* sin(y[1])+B* cos(y[0]);  // orientation of the robot
	YDOT[3]=v* cos(y[2]);
	YDOT[4]=v* sin(y[2]);
	
	if ( y[3]<= 0 || y[3]> XMAX || y[4]<= 0 || y[4] > YMAX) {   // mirror condition
    		YDOT[2]-=PI;  // to reflect the robot at boundaries
	}
     
    return YDOT; //The function returns the entire array.
}

// Runge Kutta Functions - Give an approximate solution of differential equations
/*---------------------*/

// This section is devoted to building the dynamic model of the mobile robot together with generating chaotic forcing input

double * RK4(double x, double * y, double step) {       //---------------->

	double h=step/2.0;					/* the midpoint */
	double t1[N], t2[N], t3[N];			// temporary storage arrays 
	double k1[N], k2[N], k3[N], k4[N];	// for Runge-Kutta 
	int i;

	double * yy = (double*) malloc (N*sizeof(double));
	double ret_val[N];
 
 	DE(x, y, ret_val);
 	
	for (i=0 ; i<N ; ++i)	{
		k1[i] = step * ret_val[i];	
		t1[i] = y[i] + 0.5 * k1[i];
	}

	DE(x+h, t1, ret_val);
	
	for ( i=0 ; i<N ; ++i )	{
		k2[i] = step * ret_val[i];
		t2[i] = y[i] + 0.5 * k2[i];
	}

	DE(x+h, t2, ret_val);
	
	for ( i=0 ; i<N ; ++i ) {
		k3[i] = step * ret_val[i];
		t3[i] = y[i] + k3[i];
	}
	
	DE(x+step, t3, ret_val);

	for ( i=0 ; i<N ; ++i ) {

		k4[i] = step * ret_val[i];
	}

	for ( i=0 ; i<N ; ++i ) { 

		yy[i] = y[i] + ( k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i] ) / 6.0;  // OUTPUT
	}

	return(yy);
}


// Function to return random value between 0.0 and 0.1

double  RND(void) {
	
	double rand_var;
	
	rand_var = 0.1*((double) rand () / (double) RAND_MAX); 

	return rand_var;
}
