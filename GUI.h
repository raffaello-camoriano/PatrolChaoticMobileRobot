/*  GUI THREAD: using SDL 2D graphic library,
it renders the positions of the agents on the map.*/

//Screen attributes
const int SCREEN_WIDTH = 630;
const int SCREEN_HEIGHT = 630;
const int SCREEN_BPP = 16;		//Color depth

//Initialize SDL surfaces
SDL_Surface *screen = NULL; 		//Surface of the frame
SDL_Surface *MRimg = NULL;			//Surface of the mobile robot
SDL_Surface *HAimg = NULL;			//Surface of the hostile agent
SDL_Surface *welcomeimg = NULL; 	//Surface of the welcome image
SDL_Surface *exitimg = NULL;		//Surface of the exit image
SDL_Surface *gameoverimg = NULL;	//Game over surface
SDL_Surface *gamewinimg = NULL;		//Game win surface
SDL_Surface *mapimg = NULL;			//Map boundaries surface

//Function which applies a given surface to the screen

void apply_surface( int x, int y, SDL_Surface* source, SDL_Surface* destination )
{
    //Make a temporary rectangle to hold the offsets
    SDL_Rect offset;

    //Give the offsets to the rectangle
    offset.x = x;
    offset.y = y;

    //Blit the surface
    SDL_BlitSurface( source, NULL, destination, &offset );
}

void *GUI_fun (void *p1){

	//Initialize SDL subsystems
	SDL_Init( SDL_INIT_EVERYTHING );

	printf("GUI thread working.\n");
	
	short mode;				 	 //GUI mode to be acquired at each cycle
	double pos_local [2][M];	 //Positions of the agents to be rendered (local copy)
	short alive_local[M];		 //Alive threads array (local copy)
	short i,j;					 //aux counters
	short quit = 0;				 //Flag used to quit the window
	short result;				 //Result of the simulation. 1 = game won; 0 = game over
	
	//The event structure that will be used to close the window
	SDL_Event event;

	//Initialize screen window
	screen = SDL_SetVideoMode( SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_BPP, SDL_SWSURFACE );

	//Load images
	MRimg = SDL_LoadBMP( "img/red.bmp" );
	HAimg = SDL_LoadBMP( "img/blue.bmp" );
	exitimg = SDL_LoadBMP( "img/exitimg.bmp" );
	welcomeimg = SDL_LoadBMP( "img/welcomeimg.bmp" );
	gameoverimg = SDL_LoadBMP( "img/gameoverimg.bmp" );
	gamewinimg = SDL_LoadBMP( "img/gamewinimg.bmp" );
	mapimg = SDL_LoadBMP( "img/mapimg.bmp" );
	
	//Timespec structure initialization for the nanosleep
    struct timespec pause_GUI;
    pause_GUI.tv_sec=5;
    pause_GUI.tv_nsec=0;

	apply_surface( 66, 66, welcomeimg, screen );	//display welcome screen
	
	//Update Screen
	SDL_Flip( screen );
	
	//Wait until the simulation starts
	pthread_barrier_wait (&begin_rendering_barrier);

	while (quit == 0) {			/* time loop */

		//Acquire GUI state
		pthread_mutex_lock (&GUI_mutex);
		mode = GUI_state;
		pthread_mutex_unlock (&GUI_mutex);
	
		//Mode switch

		switch (mode) {

		case 1:		//Alive, renders the position of the agents

			// Acquire positions of the agents to build next frame

			pthread_mutex_trylock (&X_mutex);
			for( i=3 ; i<5 ; ++i ){
				for ( j=0; j<M; ++j)	{
					pos_local[i-3][j] = ( X[i][j] * 50 + 65 );  		// Positions are acquired and formatted
				}
			}
			pthread_mutex_unlock (&X_mutex);

			pthread_mutex_trylock (&alive_mutex);
			for ( j=0; j<M; ++j)	{
				alive_local[j] = alive[j];  		// Agent modes are acquired
			}
			pthread_mutex_unlock (&alive_mutex);
			
			SDL_FillRect(screen, NULL, 0);	//Clears the screen
			
			//draw map boundaries
			apply_surface( 66, 66 , mapimg, screen );
			
			for ( j=0; j<M; ++j)	{
				if ( alive_local[j] == 1 )	{
					if ( j == 0 )	{
						//Render mobile robot's position in the new frame
						apply_surface( (int) floor (pos_local[0][j]), (int) floor (pos_local[1][j]) , MRimg, screen );
					}
					else	{
						//Render hostile agent's position in the new frame
						apply_surface( (int) floor (pos_local[0][j]), (int) floor (pos_local[1][j]) , HAimg, screen );
					}
				}
			}

			//Update Screen
			SDL_Flip( screen );

			break;

		case 2:		//Terminate, the simulation is finished and the threads joined

			SDL_FillRect(screen, NULL, 0);	//Clears the screen
						
			apply_surface( 66, 66 , exitimg, screen );	//apply exit message surface
			
			//Update Screen
			SDL_Flip( screen );
			
			pthread_mutex_lock (&GUI_result_mutex);
				result = GUI_result;  		// MR state is acquired
			pthread_mutex_unlock (&GUI_result_mutex);
			
			// Display result
			if ( result == 1 )	{
				apply_surface( 66, 396, gamewinimg, screen );
			}
			else {
				apply_surface( 66, 396, gameoverimg, screen );
			}
			
			//Update Screen
			SDL_Flip( screen );
			
			//Pause to display the message
			while(nanosleep(&pause_GUI,&pause_GUI)==-1)
				 continue;
			
			//Goodbye msg
			
			printf("GUI shutting down.\n");
			//Free the loaded images
			SDL_FreeSurface( MRimg );
			SDL_FreeSurface( HAimg );
			SDL_FreeSurface( exitimg );
			SDL_FreeSurface( welcomeimg );
			SDL_FreeSurface( gameoverimg );
			SDL_FreeSurface( gamewinimg );
			SDL_FreeSurface( mapimg );
			
			//Quit SDL
			SDL_Quit();
			
			//Terminate thread
			pthread_exit(NULL);

		default:
			break;
		}
	
		//While there's an SDL event to handle (Closing the window requested)

		while( SDL_PollEvent( &event ) ) {
		
			//If the user has Xed out the window
			if( event.type == SDL_QUIT ) {
				//Quit the program
				quit = 1;
			} 		
		}
	}
	
	printf("GUI shutting down.\n");
	
	//Free the loaded images
	SDL_FreeSurface( MRimg );
	SDL_FreeSurface( HAimg );
	SDL_FreeSurface( exitimg );
	SDL_FreeSurface( welcomeimg );
	SDL_FreeSurface( gameoverimg );
	SDL_FreeSurface( gamewinimg );

	//Quit SDL
	SDL_Quit();
	
	//Terminate thread
	pthread_exit(NULL);
}
