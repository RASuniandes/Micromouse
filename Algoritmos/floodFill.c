#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> // Header-file for boolean data-type.
#include "queue.h"

#define MAZE_SIZE 16

//Definiciones - Walls
#define NORTH  0x01
#define EAST   0x02
#define SOUTH  0x04
#define WEST   0x08

//mazeCells - Matriz que almacena la distancia de cada celda al centro del laberinto
unsigned char mazeCells[MAZE_SIZE][MAZE_SIZE];

//mazeWalls -  Matriz que almacena las paredes detectadas en cada celda
unsigned char mazeWalls[MAZE_SIZE][MAZE_SIZE] = { 0x0E, 0x0A, 0x09, 0x0C, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x08, 0x0A, 0x0A, 0x0A, 0x08, 0x09,
 0x0C, 0x09, 0x05, 0x06, 0x08, 0x0A, 0x0A, 0x0A, 0x0A, 0x0B, 0x06, 0x0A, 0x0A, 0x0A, 0x03, 0x05,
 0x05, 0x05, 0x05, 0x0C, 0x02, 0x0B, 0x0E, 0x08, 0x0A, 0x0A, 0x08, 0x0A, 0x08, 0x08, 0x09, 0x05,
 0x05, 0x04, 0x01, 0x06, 0x08, 0x0A, 0x09, 0x04, 0x0A, 0x0A, 0x00, 0x0A, 0x03, 0x05, 0x05, 0x05,
 0x05, 0x05, 0x04, 0x09, 0x06, 0x09, 0x05, 0x04, 0x0A, 0x0A, 0x02, 0x0A, 0x0B, 0x05, 0x05, 0x05,
 0x05, 0x04, 0x03, 0x06, 0x0A, 0x02, 0x03, 0x06, 0x0A, 0x0A, 0x0A, 0x0A, 0x09, 0x05, 0x05, 0x05,
 0x05, 0x05, 0x0D, 0x0D, 0x0D, 0x0C, 0x08, 0x0A, 0x0A, 0x0A, 0x0A, 0x09, 0x05, 0x05, 0x05, 0x05,
 0x06, 0x03, 0x04, 0x01, 0x04, 0x01, 0x05, 0x0C, 0x09, 0x0C, 0x08, 0x01, 0x05, 0x05, 0x05, 0x05,
 0x0C, 0x08, 0x01, 0x06, 0x01, 0x05, 0x04, 0x02, 0x03, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
 0x05, 0x05, 0x05, 0x0D, 0x06, 0x01, 0x05, 0x0C, 0x0A, 0x01, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
 0x05, 0x05, 0x05, 0x04, 0x09, 0x06, 0x03, 0x06, 0x0A, 0x02, 0x00, 0x03, 0x05, 0x04, 0x03, 0x05,
 0x05, 0x04, 0x03, 0x05, 0x05, 0x0C, 0x0A, 0x0A, 0x08, 0x09, 0x04, 0x0A, 0x01, 0x05, 0x0D, 0x05,
 0x05, 0x05, 0x0D, 0x05, 0x05, 0x04, 0x0A, 0x08, 0x03, 0x05, 0x06, 0x0A, 0x03, 0x05, 0x04, 0x01,
 0x05, 0x05, 0x04, 0x01, 0x04, 0x03, 0x0C, 0x02, 0x0B, 0x06, 0x08, 0x0A, 0x0A, 0x03, 0x05, 0x05,
 0x05, 0x06, 0x01, 0x07, 0x06, 0x08, 0x02, 0x0A, 0x0A, 0x0B, 0x06, 0x08, 0x0A, 0x0A, 0x00, 0x01,
 0x06, 0x0A, 0x02, 0x0A, 0x0A, 0x02, 0x0B, 0x0E, 0x0A, 0x0A, 0x0A, 0x02, 0x0A, 0x0A, 0x03, 0x07};

//mouseCell - Tupla que almacena las coordenadas actuales del MM (x,y)
unsigned char mouseCell[2];

//mouseOrientation - Variable que almacena la orientación actual del MM (Por defecto es 0)
unsigned char mouseOrientation;

//Cola de FloodFill
struct Queue* queue;

//Buffer de las distancias a los posibles vecinos
/*  
    Índice: 
        0 - Vecino en el norte
        1 - Vecino en el este
        2 - Vecino en el sur
        3 - Vecino en el oeste
*/
int neighborBuffer[4] = {-1, -1, -1, -1};

//Buffer con las coordenadas al vecino con distancia mínima
int minNeighbor[2] = {-1, -1};


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

/*Función encargada de obtener los vecinos ACCESIBLES
    Parámetrox:
        x: Posición en x de la celda central
        y: Posición en y de la celda central*/
void getNeighbors(unsigned char x, unsigned char y){
        neighborBuffer[0] = -1;
        neighborBuffer[1] = -1;
        neighborBuffer[2] = -1;
        neighborBuffer[3] = -1;

        unsigned char currentWall = mazeWalls[x][y];

        //a. Se revisa si en el Norte hay pared
        if( (currentWall & 1) != 1){
            neighborBuffer[0] = mazeCells[x][y + 1];
        }

        //b. Se revisa si en el Este hay pared
        if( (currentWall & 2) != 2){
            neighborBuffer[1] = mazeCells[x + 1][y];
        }

        //c. Se revisa si en el Sur hay pared
        if( (currentWall & 4) != 4){
            neighborBuffer[2] = mazeCells[x][y - 1];
        }

        //d. Se revisa si en Oeste hay pared
        if( (currentWall & 8) != 8){
            neighborBuffer[3] = mazeCells[x - 1][y];
        }
}

/*Función encargada de actualizar las coordenadas del vecino con la menor distancia al centro*/
void getMinNeighbor(unsigned char x, unsigned char y){

    //Por defecto, asumimos que el menor va a ser el que está al norte
    minNeighbor[0] = x + 1;
    minNeighbor[1] = y;

    unsigned char minDist = 255; //Asignamos el valor máximo de distancia

    for(char i = 0; i < 4; i++){
        if(neighborBuffer[i] != -1){
            if(neighborBuffer[i] <= minDist){
                minDist = neighborBuffer[i]; //Actualizamos la distancia mínima al vecino
                if(i == 0){
                    minNeighbor[0] = x;
                    minNeighbor[1] = y + 1;
                }
                else if(i == 1){
                    minNeighbor[0] = x + 1;
                    minNeighbor[1] = y;
                }
                else if(i == 2){
                    minNeighbor[0] = x;
                    minNeighbor[1] = y - 1;
                }
                else if(i == 3){
                    minNeighbor[0] = x - 1;
                    minNeighbor[1] = y;
                }
            }
        }
    }
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


/*Función que ejecuta el algoritmo de FloodFill*/
void updateFF(){

    //0. Buffer temporal para almacenar las coordenadas
    char cellCoords[2];

    //1. Agregar la celda actual a la cola
    unsigned char cellNum = mouseCell[0] + mouseCell[1]*16; //Se transforma de coords a valor
    enqueue(queue, cellNum);

    //2. Mientras la cola no está vacía...
    while(!isEmpty(queue)){
        cellNum = dequeue(queue);

        cellCoords[1] = cellNum / MAZE_SIZE;
        cellCoords[0] = cellNum - (MAZE_SIZE)*cellCoords[1];
        
        //3. Se obtiene la distancia de la celda actual en la que está el MM
        unsigned char current_distance  = mazeCells[cellCoords[0]][cellCoords[1]];

        //4.  Se obtienen los vecinos accesibles
        getNeighbors(cellCoords[0], cellCoords[1]); //Actualiza el buffer

        //5. Se obtienen las coordenadas del menor de los vecinos
        getMinNeighbor(cellCoords[0], cellCoords[1]); //Actualiza las coordenadas del menor de los vecinos

        //6. Si la distancia de la celda considerada es menor o igual que la distancia desde el menor de sus vecinos
        unsigned char neigh_distance = mazeCells[minNeighbor[0]][minNeighbor[1]];
        if(current_distance <= neigh_distance){

            //De cumplirse 6, se debe actualizar la distancia de la celda actual = distancia min neigh + 1
            mazeCells[cellCoords[0]][cellCoords[1]] = neigh_distance + 1;

            //Se procede a agregar todos los vecinos accesibles a la cola
            unsigned char neighNum;

            for(unsigned char i = 0; i < 4; i++){
                if(neighborBuffer[i] != -1){
                    if(i == 0){
                        neighNum = (mouseCell[0]) + (mouseCell[1] + 1)*16;
                    }
                    else if(i == 1){
                        neighNum = (mouseCell[0] + 1) + (mouseCell[1])*16;
                    }
                    else if(i == 2){
                        neighNum = (mouseCell[0]) + (mouseCell[1] - 1)*16;
                    }
                    else if(i == 3){
                        neighNum = (mouseCell[0] - 1) + (mouseCell[1])*16;
                    }

                    enqueue(queue, neighNum);
                }
            }
        }

    }

    printf("MIN) x: %d | ", minNeighbor[0]);
    printf("y: %d | ", minNeighbor[1]);
    printf("dist: %d \n", mazeCells[minNeighbor[0]][minNeighbor[1]]);

}

void main()   // define the main function  
{  

    initFF(); 
    queue = createQueue(1000);
    print_2DMatrix(mazeCells);

    //Inicialización del MM (Celda de inicio - Orientación: Norte)
    updatePosition(0, 0);
    updateOrientation(0);

    //print_2DMatrix(mazeWalls);
    

    //Simulamos el MM 
    while(mazeCells[mouseCell[0]][mouseCell[1]] != 0){
        updateFF();
        updatePosition(minNeighbor[0], minNeighbor[1]);
    }

    print_2DMatrix(mazeCells);
}  

