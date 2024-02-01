#include "stdio.h"
#include "stdlib.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include "time.h"
#include "formulas.h"
#include "wall.h"
#include "robot.h"
#include "math.h"
#include <sys/time.h>

#include <signal.h>

#include <unistd.h>
int done = 0;

void sigterm_handler(int sig) {
    exit(0);
}

int main(int argc, char *argv[]) {
    signal(SIGINT, sigterm_handler);
    SDL_Window *window;
    SDL_Renderer *renderer;
    int name_index = 0;

    if(SDL_Init(SDL_INIT_VIDEO) < 0){
        return 1;
    }

    window = SDL_CreateWindow("Robot Maze", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, OVERALL_WINDOW_WIDTH, OVERALL_WINDOW_HEIGHT, SDL_WINDOW_OPENGL);
    renderer = SDL_CreateRenderer(window, -1, 0);
    

    struct Robot robot;
    struct Wall_collection *head = NULL;
    int front_centre_sensor, left_sensor, right_sensor=0;
    // clock_t start_time, end_time;
    struct timeval stop, start; //
    int msec;
    int crashed = 0;

    // SETUP MAZE
    // You can create your own maze here. line of code is adding a wall.
    // You describe position of top left corner of wall (x, y), then width and height going down/to right
    // Relative positions are used (OVERALL_WINDOW_WIDTH and OVERALL_WINDOW_HEIGHT)
    // But you can use absolute positions. 10 is used as the width, but you can change this.

    ///////////////
    ////OWN MAZE///
    ///////////////
insertAndSetFirstWall(&head, name_index++,  640-10-150, 450, 10, 50); //x, y, x_len, y_len
    insertAndSetFirstWall(&head, name_index++,  640-10-250, 350, 10, 150);
    insertAndSetFirstWall(&head, name_index++,  640-100-50, 450, 100, 10);
    insertAndSetFirstWall(&head, name_index++,  640-100-150, 350, 100, 10);
    insertAndSetFirstWall(&head, name_index++,  640-10-50, 230, 10, 220);
    insertAndSetFirstWall(&head, name_index++,  640-10-150, 330, 10, 30);
    insertAndSetFirstWall(&head, name_index++,  640-10-50, 230, 10, 170);
    insertAndSetFirstWall(&head, name_index++,  640-10-150, 330, 10, 30);
    insertAndSetFirstWall(&head, name_index++,  640-330-50, 230, 330, 10);
    insertAndSetFirstWall(&head, name_index++,  640-130-150, 330, 130, 10);
    insertAndSetFirstWall(&head, name_index++,  640-10-380, 230, 10, 100);
    insertAndSetFirstWall(&head, name_index++,  640-10-280, 330, 10, 100);
    insertAndSetFirstWall(&head, name_index++,  640-110-280, 430, 110, 10);

    insertAndSetFirstWall(&head, name_index++,  640-150-380, 330, 150, 10);
    insertAndSetFirstWall(&head, name_index++,  640-370-280, 430, 370, 10);
    insertAndSetFirstWall(&head, name_index++,  640-10-530, 110, 10, 230);
    insertAndSetFirstWall(&head, name_index++,  640-10-630, 10, 10, 420);
    insertAndSetFirstWall(&head, name_index++,  640-20-510, 110, 20, 10);
    insertAndSetFirstWall(&head, name_index++,  640-220-410, 10, 220, 10);
    insertAndSetFirstWall(&head, name_index++,  640-10-510, 110, 10, 110);
    insertAndSetFirstWall(&head, name_index++,  640-10-410, 10, 10, 110);
    insertAndSetFirstWall(&head, name_index++,  640-230-290, 210, 230, 10);
    insertAndSetFirstWall(&head, name_index++,  640-30-390, 110, 30, 10);
    insertAndSetFirstWall(&head, name_index++,  640-10-290, 110, 10, 110);
    insertAndSetFirstWall(&head, name_index++,  640-10-390, 10, 10, 110);
    insertAndSetFirstWall(&head, name_index++,  640-240-150, 10, 240, 10);
    insertAndSetFirstWall(&head, name_index++,  640-50-250, 110, 50, 10);
    insertAndSetFirstWall(&head, name_index++,  640-10-150, 10, 10, 110);
    insertAndSetFirstWall(&head, name_index++,  640-10-250, 110, 10, 110);
    insertAndSetFirstWall(&head, name_index++,  640-50-110, 110, 50, 10);
    insertAndSetFirstWall(&head, name_index++,  640-260-0, 210, 260, 10);
    insertAndSetFirstWall(&head, name_index++,  640-10-0, 10, 10, 200);
    insertAndSetFirstWall(&head, name_index++,  640-40-0, 10, 40, 10);
    insertAndSetFirstWall(&head, name_index++,  640-10-110, 90, 10, 30);
    insertAndSetFirstWall(&head, name_index++,  640-30-110, 90, 30, 10);
    insertAndSetFirstWall(&head, name_index++,  640-10-30, 00, 10, 20);
    insertAndSetFirstWall(&head, name_index++,  640-10-130, 00, 10, 90);
// insertAndSetFirstWall(&head, name_index++,  640-10-150, 450, 10, 50); //x, y, x_len, y_len
//     insertAndSetFirstWall(&head, name_index++,  640-10-250, 350, 10, 150);
//     insertAndSetFirstWall(&head, name_index++,  640-100-50, 450, 100, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-100-150, 350, 100, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-10-50, 230, 10, 220);
//     insertAndSetFirstWall(&head, name_index++,  640-10-150, 330, 10, 30);
//     insertAndSetFirstWall(&head, name_index++,  640-10-50, 230, 10, 170);
//     insertAndSetFirstWall(&head, name_index++,  640-10-150, 330, 10, 30);
//     insertAndSetFirstWall(&head, name_index++,  640-330-50, 230, 330, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-130-150, 330, 130, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-10-380, 230, 10, 100);
//     insertAndSetFirstWall(&head, name_index++,  640-10-280, 330, 10, 100);
//     insertAndSetFirstWall(&head, name_index++,  640-110-280, 430, 110, 10);

//     insertAndSetFirstWall(&head, name_index++,  640-150-380, 330, 150, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-370-280, 430, 370, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-10-530, 110, 10, 230);
//     insertAndSetFirstWall(&head, name_index++,  640-10-630, 10, 10, 420);
//     insertAndSetFirstWall(&head, name_index++,  640-20-510, 110, 20, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-220-410, 10, 220, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-10-510, 110, 10, 110);
//     insertAndSetFirstWall(&head, name_index++,  640-10-410, 10, 10, 110);
//     insertAndSetFirstWall(&head, name_index++,  640-230-290, 210, 230, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-30-390, 110, 30, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-10-290, 110, 10, 110);
//     insertAndSetFirstWall(&head, name_index++,  640-10-390, 10, 10, 110);
//     insertAndSetFirstWall(&head, name_index++,  640-240-150, 10, 240, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-50-250, 110, 50, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-10-150, 10, 10, 110);
//     insertAndSetFirstWall(&head, name_index++,  640-10-250, 110, 10, 110);
//     insertAndSetFirstWall(&head, name_index++,  640-50-110, 110, 50, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-260-0, 210, 260, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-10-0, 10, 10, 200);
//     insertAndSetFirstWall(&head, name_index++,  640-40-0, 10, 40, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-10-110, 90, 10, 30);
//     insertAndSetFirstWall(&head, name_index++,  640-30-110, 90, 30, 10);
//     insertAndSetFirstWall(&head, name_index++,  640-10-30, 00, 10, 20);
//     insertAndSetFirstWall(&head, name_index++,  640-10-130, 00, 10, 90);

//  insertAndSetFirstWall(&head, name_index++,  640-10-0, 0, 10, 130);

//     insertAndSetFirstWall(&head, name_index++,  640-10-80, 0, 10, 30);

//     insertAndSetFirstWall(&head, name_index++,  640-30-80, 30, 30, 10);

//     insertAndSetFirstWall(&head, name_index++,  640-110-0, 130, 110, 10);




//    int i, a, b, c, d, e, f, g, h, k, l, m;

//     double j;

//  a = 30;

//     b = 30;

//     c = 10;

//  d = 3;

//  e = 110;

//  f = 130;

//  g = b;

//  h = c;

//  k = d;

//  l = e;

//  m = 90;

//     for (i = 0; i < m; i++){

//         j = i;

//         insertAndSetFirstWall(&head, name_index++,

//                               640-10-((i * d)+e),

//                               a + b*sin(c*j * M_PI/180),

//                               10, 10);

//         insertAndSetFirstWall(&head, name_index++,

//                               640-10-((i * k)+l),

//                               f + g*sin(h*j * M_PI/180),

//                               10, 10);

//     }

//     for (i = 0; i < 100; i++){

//         j = i;

//         insertAndSetFirstWall(&head, name_index++,

//                               // the most important bit is below.

//                               // increase the 20 for a tighter bend

//                               // descrease for a more meandering flow

//                               640-10-(380 + 130*sin(1.8*j * M_PI/180)),

//                               // increase the 5 for a spacier curve

//                               (i * 2)+130,

//                               10, 10);

//         insertAndSetFirstWall(&head, name_index++,

//                               // the most important bit is below.

//                               // increase the 20 for a tighter bend

//                               // descrease for a more meandering flow

//                               640-10-(380 + 230*sin(1.8*j * M_PI/180)),

//                               // increase the 5 for a spacier curve

//                               (i * 4)+30,

//                               10, 10);

//     }




//     float aa, bb;

//     a = 200;

//     aa = 0.5;

//     bb = 1;

//     c = 180;

//     d = 1;

//     e = a+100;

//     f = c;

//     m = 250;

//     for (i = 100; i < m; i++){

//         insertAndSetFirstWall(&head, name_index++,  640-10-(a + i*aa) , c + i*bb, 10, 10);

//     }

//     insertAndSetFirstWall(&head, name_index++,  640-60-330, 430, 60, 10);

//     m = 150;

//     for (i = 0; i < m; i++){

//         insertAndSetFirstWall(&head, name_index++,  640-10-(e + i*aa) , f + i*bb, 10, 10);

//     }

//     insertAndSetFirstWall(&head, name_index++,  640-150-150, 180, 150, 10);

//     e = a-50;

//     aa = 0.25;

//     for (i = 0; i < m; i++){

//         insertAndSetFirstWall(&head, name_index++,  640-10-(e - i*aa) , f + i*bb, 10, 10);

//     }

//     a=a+50;

//     c=c+100;

//     aa = 0.75;

//     for (i = 0; i < m; i++){

//         insertAndSetFirstWall(&head, name_index++,  640-10-(a - i*aa) , c + i*bb, 10, 10);

//     }

//     insertAndSetFirstWall(&head, name_index++,  640-100-40, 430, 100, 10);

//     insertAndSetFirstWall(&head, name_index++,  640-10-40, 240, 10, 190);

//     insertAndSetFirstWall(&head, name_index++,  640-10-110, 180, 10, 160);

//     insertAndSetFirstWall(&head, name_index++,  640-40-0, 240, 40, 10);

//     insertAndSetFirstWall(&head, name_index++,  640-110-0, 180, 110, 10);
   
////////////////////////////////////////////////////////////////////////


    
    setup_robot(&robot);
    updateAllWalls(head, renderer);

    SDL_Event event;
    while(!done){
        SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
        SDL_RenderClear(renderer);

        //Move robot based on user input commands/auto commands
        if (robot.auto_mode == 1)
            robotAutoMotorMove(&robot, front_centre_sensor, left_sensor, right_sensor);
        robotMotorMove(&robot, crashed);

    // if (checkRobotReachedEnd(&robot, OVERALL_WINDOW_WIDTH - 30, OVERALL_WINDOW_HEIGHT -92, 10, 82)){ // OWN MAZE, THIS IS THE LOCATION OF THE END 
    if (checkRobotReachedEnd(&robot, 640-110-30, 0, 100, 10)){ // Maze 3
    // if (checkRobotReachedEnd(&robot, 150, 470, 100, 10)){ //Maze 2
    // if (checkRobotReachedEnd(&robot, 640-110-30, 470, 100, 10)){ //Maze 4
    // if (checkRobotReachedEnd(&robot, 640-10-90, 0, 90, 10)){ //Maze 8
            // end_time = clock();
            // msec = ((end_time-start_time)*1000 )/ CLOCKS_PER_SEC;
            gettimeofday(&stop, NULL); //
            int msec = (stop.tv_sec - start.tv_sec) * 1000 + (stop.tv_usec - start.tv_usec) / 1000; //
            robotSuccess(&robot, msec);

            if (robot.crashed) {
            struct Wall_collection *current = head;
            while (current != NULL) {
                struct Wall_collection *temp = current;
                current = current->next;
                free(temp);
            }
            head = NULL;
            // EZ that will apear when robot finish//
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
            insertAndSetFirstWall(&head, 1,  OVERALL_WINDOW_WIDTH/6, OVERALL_WINDOW_HEIGHT/4, 10, OVERALL_WINDOW_HEIGHT/2);
            insertAndSetFirstWall(&head, 2,  OVERALL_WINDOW_WIDTH/6, OVERALL_WINDOW_HEIGHT/4, OVERALL_WINDOW_WIDTH/4, 10);
            insertAndSetFirstWall(&head, 3,  OVERALL_WINDOW_WIDTH/6, OVERALL_WINDOW_HEIGHT/2, OVERALL_WINDOW_WIDTH/4, 10);
            insertAndSetFirstWall(&head, 4,  OVERALL_WINDOW_WIDTH/6, 3*OVERALL_WINDOW_HEIGHT/4, OVERALL_WINDOW_WIDTH/4, 10);
            insertAndSetFirstWall(&head, 5,  7*OVERALL_WINDOW_WIDTH/12, OVERALL_WINDOW_HEIGHT/4, OVERALL_WINDOW_WIDTH/4, 10);
            double dx = cos(57 * M_PI / 180.0);
            double dy = sin(57 * M_PI / 180.0);
            for (int i = 0; i < 285; i++) {
                insertAndSetFirstWall(&head, 7+i+1, 7*OVERALL_WINDOW_WIDTH/12 + dx*i, 3*OVERALL_WINDOW_HEIGHT/4 - i*dy, 8, 8);
            }
            insertAndSetFirstWall(&head, 6,  7*OVERALL_WINDOW_WIDTH/12, 3*OVERALL_WINDOW_HEIGHT/4, OVERALL_WINDOW_WIDTH/4, 10);

            updateAllWalls(head, renderer);
            // Define an array of background colors
            SDL_Color backgroundColors[] = {
                {0, 0, 255, 255},   // Blue
                {0, 255, 0, 255},   // Green
                {139, 69, 19, 255}, // Brown
                {255, 192, 203, 255}, // Pink
                {255, 0, 0, 255},   // Red
                {255, 255, 0, 255}, // Yellow
                {255, 165, 0, 255}  // Orange
            };
            Uint32 lastColorChangeTime = 0;
            Uint32 colorChangeInterval = 500;  
            int currentColorIndex = 0;
            while (!done) {
                Uint32 currentTime = SDL_GetTicks();
                Uint32 elapsedTime = currentTime - lastColorChangeTime;
                if (elapsedTime >= colorChangeInterval) {
                    lastColorChangeTime = currentTime;
                    currentColorIndex = (currentColorIndex + 1) % (sizeof(backgroundColors) / sizeof(backgroundColors[0]));
                }
                SDL_Color currentBackgroundColor = backgroundColors[currentColorIndex];
                SDL_SetRenderDrawColor(renderer, currentBackgroundColor.r, currentBackgroundColor.g, currentBackgroundColor.b, currentBackgroundColor.a);
                SDL_RenderClear(renderer);
                updateAllWalls(head, renderer);
                SDL_RenderPresent(renderer);
                SDL_Delay(120);
            }
        }

        }
        else if(crashed == 1 || checkRobotHitWalls(&robot, head)){
            robotCrash(&robot);
            crashed = 1;
        }
        //Otherwise compute sensor information
        else {
            front_centre_sensor = checkRobotSensorFrontCentreAllWalls(&robot, head);
            if (front_centre_sensor>0)
                printf("Getting close on the centre. Score = %d\n", front_centre_sensor);

            left_sensor = checkRobotSensorLeftAllWalls(&robot, head);
            if (left_sensor>0)
                printf("Getting close on the left. Score = %d\n", left_sensor);

            right_sensor = checkRobotSensorRightAllWalls(&robot, head);
            if (right_sensor>0)
                printf("Getting close on the right. Score = %d\n", right_sensor);
        }
        robotUpdate(renderer, &robot);
        updateAllWalls(head, renderer);

        // Check for user input
        SDL_RenderPresent(renderer);
        while(SDL_PollEvent(&event)){
            if (event.type == SDL_WINDOWEVENT) {
                if (event.window.event == SDL_WINDOWEVENT_CLOSE) {
                    done = 1;
                }
            }
            if(event.type == SDL_QUIT){
                done = 1;
            }
            const Uint8 *state = SDL_GetKeyboardState(NULL);
            if(state[SDL_SCANCODE_UP] && robot.direction != DOWN){
                robot.direction = UP;
            }
            if(state[SDL_SCANCODE_DOWN] && robot.direction != UP){
                robot.direction = DOWN;
            }
            if(state[SDL_SCANCODE_LEFT] && robot.direction != RIGHT){
                robot.direction = LEFT;
            }
            if(state[SDL_SCANCODE_RIGHT] && robot.direction != LEFT){
                robot.direction = RIGHT;
            }
            if(state[SDL_SCANCODE_SPACE]){
                setup_robot(&robot);
                crashed = 0;
            }
            if(state[SDL_SCANCODE_RETURN]){
                robot.auto_mode = 1;
                // start_time = clock();
                gettimeofday(&start, NULL); //
            }
            if (state[SDL_SCANCODE_ESCAPE]) done = 1;
        }

        SDL_Delay(120);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    printf("DEAD\n");
}
