#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_STATUS_REQUEST
#define _TASK_PRIORITY
#include <TaskScheduler.h>

#include <Servo.h>
#include "ROSBraccio.h"

StatusRequest reading, st;

Scheduler ts;
Scheduler hpts;

bool debug=false;

// Joints declaration
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

// Msg variables
const byte numChars = 35;
char receivedChars[numChars];
char sentChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
boolean newData = false;
char operMode[1] = {0};
const byte numJoints = 6;

int joint_states[numJoints]={90, 45, 180, 180, 90, 10};

// Callback methods prototypes
void CallbackReadSerial(); void EnableT1();void DisableT1();
void CallbackParseData(); bool EnableT2(); void DisableT2();
void CallbackCommandRobot(); bool EnableT3();
void CallbackSendState(); bool EnableT4();
void CallbackSendAlive(); bool EnableT5();

// Tasks
//   T1: Reads serial. High priority task.
//   T2: Parses message. High priority task.
//   T3: Sends commands to robot. Low priority.
//   T4: Sends robot state to ROS (feedback). High priority.
//   T5: Sends signal that Arduino is alive. High priority.
Task t1(20, TASK_FOREVER, &CallbackReadSerial, &hpts, true, NULL, &DisableT1);
Task t2(0, TASK_ONCE, &CallbackParseData, &hpts, false, &EnableT2);
Task t3(0, TASK_ONCE, &CallbackCommandRobot, &ts, false, &EnableT3);
Task t4(0, TASK_ONCE, &CallbackSendState, &hpts, false, &EnableT4);
Task t5(0, TASK_ONCE, &CallbackSendAlive, &hpts, false, &EnableT5);

/** T1 callback
 *  T1 gathers serial data
 */
void CallbackReadSerial() {
  if(debug)
  {
    Serial.println("T1: Signaling completion of ST");
  }

  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
      rc = Serial.read();

      if (recvInProgress == true) {
          if (rc != endMarker) {
              receivedChars[ndx] = rc;
              ndx++;
              if (ndx >= numChars) {
                  ndx = numChars - 1;
              }
          }
          else {
              receivedChars[ndx] = '\0'; // terminate the string (received data)
              recvInProgress = false;
              ndx = 0;
              newData = true;

              if(debug)
              {
                Serial.print("Received: ");
                Serial.print(receivedChars);
                Serial.print("\n");
              }

              t2.restartDelayed();
          }
      }

      else if (rc == startMarker) {
          recvInProgress = true;
      }
  }

}

/** T1 On Disable callback
 *  This callback renews the status request and restarts T1 delayed to run again in 5 seconds
 */
void DisableT1() {
  //PrepareStatus();
  t1.restartDelayed();
  t2.waitFor(&reading);
}

bool EnableT2(){
  return true;
}

/** T2 callback
 *  Invoked when status request st completes (T1) and parses data
 */
void CallbackParseData() {
  if(debug)
  {
    Serial.println("T2: Invoked due to complete message received succesfully (T1)");
    Serial.println("Parseing data...");
  }

  strcpy(tempChars, receivedChars);// this temporary copy is necessary to protect the original data because strtok() used in parseData() replaces the commas with \0

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars,",");// get the first part - the string
  strcpy(operMode, strtokIndx);// copy it inputMode

  if (operMode[0] == 'w')
  {
    for (int i = 0; i < numJoints; i++)
    {
      strtokIndx = strtok(NULL, ",");// this continues where the previous call left off
      joint_states[i] = atoi(strtokIndx); // convert this part to an integer in joint_states
    }
  }
  
  switch (operMode[0]){
    case 'w':
      t3.restartDelayed();
      break;
    case 'r':
      t4.restartDelayed();
      break;
    case 'l':
      t5.restartDelayed();
      break;
  }
  newData = false;

}

bool EnableT3(){
  return true;
}

/** T3 callback
 *  Invoked when status request st completes (T1).
 */
 void CallbackCommandRobot() {
  if(debug)
  {
    Serial.println("T3: Invoked due to completion of T2->writing command");  
  }

  Braccio.ServoMovement(joint_states[0],  joint_states[1], joint_states[2], joint_states[3], joint_states[4],  joint_states[5]);  
}

bool EnableT4(){
  return true;
}

/** T4 callback
 *  Invoked when status request st completes (T1).
 */
 void CallbackSendState() {
  if(debug)
  {
    Serial.println("T4: Invoked due to completion of T2->sending status");  
  }

  if(debug)
  {
    Serial.print("Message: ");
    Serial.println(operMode);
    
    for (int i = 0; i < numJoints; i++)
    {
      Serial.print("joint state "); Serial.print(i); Serial.print(" :");
      Serial.println(joint_states[i]);
    }
  }

  compileMsg(sentChars, joint_states, numJoints);
  Serial.write(sentChars);
}

bool EnableT5(){
  return true;
}

/** T4 callback
 *  Invoked when status request st completes (T1).
 */
 void CallbackSendAlive() {
  if(debug)
  {
    Serial.println("T5: Invoked due to completion of T2->sending alive status");  
  }

  if(debug)
  {
    Serial.print("Message: ");
    Serial.println(operMode);
    

  }

  Serial.write("<ok>");
}



void compileMsg(char* Msg, int* j_States, int n_joints)
{
  String str;
  
  for (int i = 0; i < n_joints; i++)
  {
    if (i==0)
    {
      str = String("<r,");
      str = str + String(j_States[i]);
      str = str + ",";
    }
    else if (i == (n_joints-1))
    {
      str = str + String(j_States[i]);
      str = str + ">";
      str = str + "\0";
    }
    else
    {
      str = str + String(j_States[i]);
      str = str + ",";
    }
  
    str.toCharArray(Msg, str.length()+1);
  }
}

/** Main Arduino code
 *  Not much to do here. Just init Serial and set the initial status request
 */
void setup() {

  Serial.begin(115200);
  while(!Serial);

  // Braccio setup
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  //Braccio.begin();
  Braccio.begin(SOFT_START_DISABLED);

  //Serial.println("TaskScheduler: Status Request Test 1. Simple Test.");  

  //ts.startNow();
  ts.setHighPriorityScheduler(&hpts);
  ts.enableAll(true);

  t1.restartDelayed();
}

void loop() {

  ts.execute();

}