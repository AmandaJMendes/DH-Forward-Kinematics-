#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#define n 4 //Number of joints


typedef struct _Joint{
  //Definition of joint with DH parameters
  int rotary;
  double theta;
  double alpha;
  double r;
  double d;
  double **matrix;
} Joint;


double to_radians(double angle){
  //This function converts degrees to radians
  return angle*M_PI/180;
}


double to_degrees(double angle){
  //This function converts radians to degrees    
  return angle*180/M_PI;
}


void printMatrix(double **matrix, int rows, int cols){
  //This function prints a matrix
  for (int i = 0; i < rows; i++){
    for (int j = 0; j < cols; j++){
      printf("%lf ",matrix[i][j]);
    }
    printf("\n");
  }
}


double **dot(double **m1, double **m2, int d){
  //This function returns the dot product between two dxd matrices
  double sum;
  double **y;

  y = (double **)malloc(d * sizeof(double *)); 
  
  for (int row = 0; row < d; row++) {
    y[row] = (double *)malloc(d * sizeof(double));
  }

  for (int i = 0; i < d; i++){ 
    for (int j = 0; j < d; j++){
      sum = 0;
      for (int k = 0; k < d; k++){
        sum = sum + m1[i][k] * m2[k][j];
      }
      y[i][j] = sum;
    } 
  }

  return y;
}


void DHMatrix(Joint *j){
  //This function creates a DH matrix for a joint j
  j->matrix = (double **)malloc(4 * sizeof(double *)); 
  
  for (int row = 0; row < 4; row++) {
    j->matrix[row] = (double *)malloc(4 * sizeof(double));
  }
  
  j->matrix[3][0] =  j->matrix[3][1] = j->matrix[3][2] = j->matrix[2][0] = 0;
  j->matrix[3][3] = 1;
  j->matrix[2][1] = sin(j->alpha);
  j->matrix[2][2] = cos(j->alpha);

  j->matrix[0][0] = cos(j->theta);
  j->matrix[1][0] = sin(j->theta);

  j->matrix[0][1] = -j->matrix[1][0]*j->matrix[2][2];
  j->matrix[1][1] = j->matrix[0][0]*j->matrix[2][2];

  j->matrix[0][2] = j->matrix[1][0]*j->matrix[2][1];
  j->matrix[1][2] = -j->matrix[0][0]*j->matrix[2][1];

  j->matrix[0][3] = j->r * j->matrix[0][0];
  j->matrix[1][3] = j->r * j->matrix[1][0];

  j->matrix[2][3] = j->d;
}


void updateDHMatrix(Joint *j){
  //This function updates the variable elements of the joint's matrix
  if (j->rotary){
    j->matrix[0][0] = cos(j->theta);
    j->matrix[1][0] = sin(j->theta);

    j->matrix[0][1] = -j->matrix[1][0]*j->matrix[2][2];
    j->matrix[1][1] = j->matrix[0][0]*j->matrix[2][2];

    j->matrix[0][2] = j->matrix[1][0]*j->matrix[2][1];
    j->matrix[1][2] = -j->matrix[0][0]*j->matrix[2][1];

    j->matrix[0][3] = j->r * j->matrix[0][0];
    j->matrix[1][3] = j->r * j->matrix[1][0];
  }
  
  j->matrix[2][3] = j->d;
  
}


double **updateKinChain(Joint **js){
  //This function returns a matrix containing the coordinates of the effectuator with respect to the base
  double **mult, **aux;
  mult = dot(js[0]->matrix, js[1]->matrix, 4);
  for (int i = 2; i < n; i++){
    aux = mult;
    mult = dot(aux, js[i]->matrix, 4);
    free(aux);
  }
  return mult; 
}


void updateJoint(Joint *j, double value, int increment){
  //This functions updates the joint's value
  //increment = True --> the joint's value will be incremented by value parameter
  //increment = False --> the joint's value will be set to value parameter
  if (j->rotary){
    if (increment){
      j->theta = j->theta + to_radians(value);
    }
    else{
      j->theta = to_radians(value);
    }
  }
  else{
    if (increment){
      j->d = j->d + value;
    }
    else{
      j->d = value;
    }
  }
  updateDHMatrix(j);
}

void printPosition(Joint **joints, double **matrix){
  //This function prints the effectuator's position (x, y, z) with respect to the robot's base
  //Also, it prints the value of each joint
  printf("End-effectuator position: (%.2lf, %.2lf, %.2lf)\n\
Joints' positions:\n\
Joint 1: %.2lf cm\n\
Joint 2: %.2lf°\n\
Joint 3: %.2lf°\n",
  matrix[0][3], matrix[1][3], matrix[2][3],
  joints[0]->d, to_degrees(joints[1]->theta),
  to_degrees(joints[2]->theta));
}

int main() {
  int i0, i1, i2; //Instructions from the user
  double i3; //Instructions from the user
  double q1, q2, q3; //Joints' initial positions
  Joint j1, j2, j3, j4; //Robot's joints
  Joint **js; //Vector with all the robot's joints
  double **tMatrix; //Variable to store updateKinChain() output
  int config = 0;

  js = (Joint **)malloc(n*sizeof(Joint *));

  js[0] = &j1;
  js[1] = &j2;
  js[2] = &j3;
  js[3] = &j4;
  
  j1.rotary = 0;
  j2.rotary = j3.rotary = j4.rotary = 1;

  j1.alpha = j2.alpha = j3.alpha = j4.alpha = 0;
  j1.theta = j4.theta = 0;
  j2.d = j3.d = 0;
  j1.r = j4.r = 0;

  
  DHMatrix(&j1);
  DHMatrix(&j2);
  DHMatrix(&j3);
  DHMatrix(&j4);
      
  printf("Type 0, 1 or 2:\n- 0: exit.\n- 1: configure a robot.\n- 2: control the robot.\n");
  scanf("%d", &i0);

  while (i0 != 0){ //i0=0 -> End
  
    if (i0==1){ //Configure

      if (config == 0){
        config = 1;
      }
      
      j2.r = j3.r = j4.d = -1; //Temporary value
      
      //Get robot's parameters from the user
      while (j2.r <= 0){
        printf("Length of the first link in cm: ");
        scanf("%lf", &j2.r);
        if (j2.r <= 0){
          printf("Length must be greater than 0.\n");
        }
      }
      
      while (j3.r <= 0){
        printf("Length of the second link in cm: ");
        scanf("%lf", &j3.r);
        if (j3.r <= 0){
          printf("Length must be greater than 0.\n");
        }
      }
      
      while (j4.d <= 0){
        printf("Length of the third link in cm: ");
        scanf("%lf", &j4.d);
        if (j4.d <= 0){
          printf("Length must be greater than 0.\n");
        }
      }
      j4.d = -j4.d;
      
      printf("Initial height in cm (prismatic joint): ");
      scanf("%lf", &q1);
      printf("Initial angle of the second joint in degrees (rotary joint): ");
      scanf("%lf", &q2);
      printf("Initial angle of the third joint in degrees (rotary joint): ");
      scanf("%lf", &q3);

      updateJoint(js[0], q1, 0);
      updateJoint(js[1], q2, 0);
      updateJoint(js[2], q3, 0);
      //The 4th joint's theta is fixed at 0 and its d parameter (z-offset) is defined by the user. 
      //After configuration, this  z-offset can't be altered.
      updateDHMatrix(js[3]);  

      tMatrix = updateKinChain(js); 
      
      printf("\nThe robot was configured successfully. ");
      printPosition(js, tMatrix);
      free(tMatrix); //Release the memory allocated in updateKinChain fucntion
    }

    else if (i0 == 2){ //Control robot 
      if (config == 0){
        printf("There is no configured robot.\n");
      }
      else{
        printf("\nType 0, 1, 2 or 3:\n- 0: return.\n- 1: increment/decrement joint's position.\n- 2: define joint's position. \n- 3: reset positions.\n");
        scanf("%d", &i1);

        while (i1 != 0){ //Return

          if (i1 == 3){ //Reset robot

            updateJoint(js[0], q1, 0);
            updateJoint(js[1], q2, 0);
            updateJoint(js[2], q3, 0);
  
            tMatrix = updateKinChain(js);
            printf("\nThe robot was reset successfully. ");
            printPosition(js, tMatrix); 
            free(tMatrix);
          }

          else if (i1 == 1 || i1 == 2){
            printf("\nJoint (1, 2, or 3): ");
            scanf("%d", &i2);
            if (i2 == 1 || i2 == 2 || i2 == 3){
              if (i1 == 1){ //Increment position
                if (i2 == 1){ //Prismatic
                  printf("How many cm? ");
                  scanf("%lf", &i3);
                }
                else{ //Rotary
                  printf("How many degrees? ");
                  scanf("%lf", &i3);            
                }

                updateJoint(js[i2-1], i3, 1);
              }

              if (i1 == 2){ //Define position
                if (i2 == 1){ //Prismatic
                  printf("New height (cm): ");
                  scanf("%lf", &i3);
                }
                else{ //Rotary
                  printf("New angle (°): ");
                  scanf("%lf", &i3);    
                }

                updateJoint(js[i2-1], i3, 0);            
              }

              
              tMatrix = updateKinChain(js);
              printPosition(js, tMatrix);
              free(tMatrix);
            }
            else{
              printf("Invalid input.\n");
            }
          }

          else{
            printf("Invalid input.\n");
          }

          printf("\nType 0, 1, 2 or 3:\n- 0: return.\n- 1: increment/decrement joint's position.\n- 2: define joint's position. \n- 3: reset positions.\n");
          scanf("%d", &i1);
        }
      }
    }
    else{
      printf("Invalid input.\n");
    }

    printf("\nType 0, 1 ou 2:\n- 0: exit.\n- 1: configure a robot.\n- 2: control the robot.\n");
    scanf("%d", &i0);
    
  }

  
  
  for (int i = 0; i < n; i++){ //Release the memory allocated for the joints' matrices
    for (int row = 0; row < 4; row++) {
      free(js[i]->matrix[row]);
    }
    free(js[i]->matrix);
  }
  
  free(js); //Release the memory allocated for the joints' vector 

  return 0;
}
