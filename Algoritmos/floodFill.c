#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> // Header-file for boolean data-type.

#define MAZE_SIZE 16

//Definiciones - Walls
#define NORTH  0x01
#define EAST   0x02
#define SOUTH  0x04
#define WEST   0x08

//mazeCells - Matriz que almacena la distancia de cada celda al centro del laberinto
unsigned char mazeCells[MAZE_SIZE][MAZE_SIZE];

//mazeWalls -  Matriz que almacena las paredes detectadas en cada celda
unsigned char mazeWalls[MAZE_SIZE][MAZE_SIZE];

//mouseCell - Tupla que almacena las coordenadas actuales del MM (x,y)
unsigned char mouseCell[2];

//mouseOrientation - Variable que almacena la orientación actual del MM (Por defecto es 0)
unsigned char mouseOrientation;

/*Función de ayuda para imprimir matrices cuadradas*/
void print_2DMatrix(unsigned char matrix[MAZE_SIZE][MAZE_SIZE]){
    for(unsigned char i = 0; i < MAZE_SIZE; i++){
        for(unsigned char j = 0; j < MAZE_SIZE; j++){
            printf("%d, ", matrix[i][j]);
        }
        printf("\n");
    }
}


/*Función encargada de actualizar las paredes vistas en cada celda:
    Requiere:
        1) Posición actual del MM
        2) Orientación del MM
        3) Detección sensórica*/
void updateWalls(bool isLeftWall, bool isRightWall, bool isFrontWall){
    
    char currentWall = 0b0000;

    //Se determina donde están las paredes según la orientación del MM
    if(mouseOrientation == 0){
        currentWall = (isFrontWall)*NORTH + (isRightWall)*EAST + (isLeftWall)*WEST;
    }
    else if(mouseOrientation == 1){
        currentWall = (isFrontWall)*EAST + (isRightWall)*SOUTH + (isLeftWall)*NORTH;
    }
    else if(mouseOrientation == 2){
        currentWall = (isFrontWall)*SOUTH + (isRightWall)*WEST + (isLeftWall)*EAST;
    }
    else if(mouseOrientation == 3){
        currentWall = (isFrontWall)*WEST + (isRightWall)*NORTH + (isLeftWall)*SOUTH;
    }


    //Se agrega la pared de abajo si el MM se encuentra en el inicio
    if(mouseCell[0] == 0 && mouseCell[1] == 0){
        currentWall += SOUTH;
    }


    //Se actualiza la matriz de paredes
    mazeWalls[mouseCell[0]][mouseCell[1]] = currentWall;
}

/*Función encargada de actualizar la orientación del MM:
    0 - ↑
    1 - →
    2 - ↓
    3 - ←
*/
void updateOrientation(unsigned char orientation){
    mouseOrientation = orientation; 
}

/*Función encargada de actualizar la posición del MM*/
void updatePosition(unsigned char x, unsigned char y){
    mouseCell[0] = x;
    mouseCell[1] = y;
}



/* Función que inicializa el algoritmo de FloodFill, asignando los respectivos valores a la matriz mazeCells */
void initFF(){
    char fill_array[MAZE_SIZE];
    char farr_index = 0;

    //a. Distancias por defecto de FloodFill (No-Walls)
    for(char i = (MAZE_SIZE/2)-1; i <= MAZE_SIZE - 2; i++){
        fill_array[(MAZE_SIZE/2) - farr_index - 1] = i;
        fill_array[i+1] = i;
        farr_index++;
    }
    for(char i = 0; i < MAZE_SIZE; i++){
        for(char j = 0; j < MAZE_SIZE; j++){
            if(i < MAZE_SIZE/2){
                mazeCells[i][j] = fill_array[j] - i;
            }
            else{
                mazeCells[i][j] = fill_array[j] - (MAZE_SIZE - i - 1);
            }
        }
    }

    //b. Ubicación por defecto del micromouse (Esquina inferior izquierda)
    mouseCell[0] = 0; 
    mouseCell[1] = 0;
}

void main()   // define the main function  
{  
    initFF(); 
    //print_2DMatrix(mazeCells);

    //Inicialización del MM (Celda de inicio - Orientación: Norte)
    updatePosition(0, 0);
    updateOrientation(0);
    updateWalls(true, true, false);

    print_2DMatrix(mazeWalls);
}  