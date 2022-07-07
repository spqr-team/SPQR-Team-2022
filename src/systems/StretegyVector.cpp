// strategy states circular vector 
#include <Arduino.h>
#include "systems\StrategyVector.h"

// strategy vector
int statesVector[VECTOR_SIZE] = {0};

// current index (head)
int currentIndex = VECTOR_SIZE - 1;

// last index (tail) 
int lastIndex = 0; 

// previous index 
int Index_prec = 0; 


// update the strategy vector with a new state
void updateVector(int newState) {
    // currentIndex circular array
    currentIndex = (currentIndex + 1)  %  VECTOR_SIZE;
    // update 
    statesVector[currentIndex] = newState;
    // reset the previous index
    Index_prec = currentIndex; 
}


// returns the previos state in the strategy vector
int getPreviousState() {
    if(Index_prec == 0) Index_prec = VECTOR_SIZE - 1;
    else Index_prec--;
    return statesVector[Index_prec];
}


// reset the strategy vector
void resetVector() {
  for(int i = 0; i < VECTOR_SIZE; i++) statesVector[i] = 0;
}