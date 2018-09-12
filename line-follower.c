////////////////////////////////////////////////////////////////////////////////

#define DEBUG

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

// 
//  -------------------------------------
//    x       x       x       x       x
//  -------------------------------------
//    0       1       2       3       4
//
// SEN_NOT_VALID - No brightest found 
// (all values less than or greater than the threshold)

#define MIDDLE 2
#define RIGHT 3
#define RRIGHT 4
#define LEFT 1
#define LLEFT 0
#define DROP_STOP 6

////////////////////////////////////////////////////////////////////////////////

// Motor pins
#define PIN_MOTOR_RIGHT_FORWARD 10
#define PIN_MOTOR_RIGHT_REVERSE 11
#define PIN_MOTOR_LEFT_FORWARD 3
#define PIN_MOTOR_LEFT_REVERSE 9

// Headlight pins
#define PIN_HEADLIGHT_RIGHT 4     
#define PIN_HEADLIGHT_LEFT 5    

// Speed
#define SPEED_MAX 220

// Sensors
#define SEN_NUM_SENSORS 5
// Readings smaller than this are considered dark
#define SEN_DARK_THRESHOLD 800
// If the average sensor reading (over all sensors) is smaller than this
// we conclude the device is being lifted off the ground
#define SEN_ON_AIR_THRESHOLD 90
// Number of iterations the average sensor reading is counted over
#define SEN_ITERATIONS 3
#define SEN_NOT_VALID_INDEX 5

unsigned int g_sensors_array[SEN_NUM_SENSORS];

// Lost timeouts are not any real units. Larger values simply mean
// longer times
// How long do we keep going after the track is lost?
#define TRACK_LOST_TIMEOUT 200
// Stop after having been lost long enough
#define COMPLETELY_LOST_TIMEOUT 800

// Steering
unsigned short int g_newdirection = MIDDLE;
unsigned short int g_lastdirection = MIDDLE;
unsigned short int g_lastturn = RRIGHT;
unsigned int g_tracklost = 0;


////////////////////////////////////////////////////////////////////////////////

void setup() {

#ifdef DEBUG
  Serial.begin(9600);
#endif

  delay(1000);

  // initialize the motor pins as output
  pinMode(PIN_MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_REVERSE, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_REVERSE, OUTPUT);  

  // Initialize the LED headlight pin as an output
  pinMode(PIN_HEADLIGHT_RIGHT, OUTPUT);
  pinMode(PIN_HEADLIGHT_LEFT, OUTPUT);

  // Turn on headlights
  digitalWrite(PIN_HEADLIGHT_RIGHT, LOW);
  digitalWrite(PIN_HEADLIGHT_LEFT, LOW);

  setspeed(0,0);
}

////////////////////////////////////////////////////////////////////////////////
  
void loop() {
  navigate();
  //delay(100);
  steer();
#ifdef DEBUG
  debug();
#endif
}

////////////////////////////////////////////////////////////////////////////////

void navigate() {
  unsigned int dark_threshold = (SEN_ITERATIONS * SEN_DARK_THRESHOLD);
  unsigned int on_air_threshold = (SEN_ITERATIONS * SEN_ON_AIR_THRESHOLD);
  unsigned int selectedvalue = dark_threshold;
  unsigned int currentvalue = dark_threshold;
  unsigned int sumsensors = 0;
  
  // For each reading count sum of SEN_ITERATIONS readings
  for (int j = 0; j < SEN_ITERATIONS; j++) {
    if (j == 0) {
      for (int i = 0; i < SEN_NUM_SENSORS; i++) {
        g_sensors_array[i] = analogRead(i);
      }
    }
    else {
      for (int i = 0; i < SEN_NUM_SENSORS; i++) {
        g_sensors_array[i] += analogRead(i);
      }      
    }
  }
  
  // Attempt to select one of the sensors
  for (int i = 0; i < SEN_NUM_SENSORS; i++) {
    currentvalue = g_sensors_array[i];
    sumsensors += currentvalue;
    if (currentvalue < selectedvalue) {
      selectedvalue = currentvalue;
      g_newdirection = i;
    }
  }
  if (sumsensors <= on_air_threshold) {
    g_newdirection = DROP_STOP;
    return; // return immediately
  }
  else if (selectedvalue >= dark_threshold) {
    g_newdirection = SEN_NOT_VALID_INDEX;
  }
  // Is this a new turn?
  if (g_newdirection != g_lastturn && 
      g_newdirection != MIDDLE && 
      g_newdirection != SEN_NOT_VALID_INDEX) {
    g_lastturn = g_newdirection;
  }
  // Is this a new direction?
  if ( g_newdirection != SEN_NOT_VALID_INDEX) {
    g_lastdirection = g_newdirection;
  }
}

////////////////////////////////////////////////////////////////////////////////

void steer() {
  double straight = 1.0;
  double slightturn = 0.4*straight;
  double fullturn = 0.0;
  int right = 0;
  int left = 0;

  if (g_newdirection == DROP_STOP) {
    setspeed(0,0);
    return;
  }
  else if (g_newdirection == SEN_NOT_VALID_INDEX) {
    losttrack();
  }
  else {
    g_tracklost = 0;
  }
  if(g_newdirection == MIDDLE) {
    // Go straight
    right = (int)(SPEED_MAX*straight);
    left = right; 
  }
  else if (g_newdirection == LEFT) {
    // Turn slightly left
    right = (int)(SPEED_MAX*straight);
    left = (int)(SPEED_MAX*slightturn);
  }
  else if (g_newdirection == LLEFT) {
    // Turn full left
    right = (int)(SPEED_MAX*straight);
    left = (int)(SPEED_MAX*fullturn);
  }
  else if (g_newdirection == RIGHT) {
    // Turn slightly right
    right = (int)(SPEED_MAX*slightturn);
    left = (int)(SPEED_MAX*straight);
  }
  else if (g_newdirection == RRIGHT) {
    // Turn full right
    right = (int)(SPEED_MAX*fullturn);
    left = (int)(SPEED_MAX*straight);
  }
  setspeed(left, right);
}

////////////////////////////////////////////////////////////////////////////////

void losttrack() {
  ++g_tracklost;
  
  unsigned int speedcorrtracklost = (int)(g_tracklost*SPEED_MAX*0.1);
  if(speedcorrtracklost < TRACK_LOST_TIMEOUT) {
    // We consider this a temporary tracklost: keep going the same direction
    g_newdirection = g_lastdirection;
  }
  else if(speedcorrtracklost >= TRACK_LOST_TIMEOUT &&
          speedcorrtracklost < COMPLETELY_LOST_TIMEOUT ) {
    // If we've been lost long enough, try to find back on track
    // What was the last recorded turn? Take the same turn again
    g_newdirection = g_lastturn;
  } 
  else if(speedcorrtracklost >= COMPLETELY_LOST_TIMEOUT) {
    // Stop if we are completely lost
    setspeed(0,0);
  }
}

////////////////////////////////////////////////////////////////////////////////

void setspeed(int left, int right) {
  if (right < 0) {
    analogWrite(PIN_MOTOR_RIGHT_REVERSE,abs(right));
    digitalWrite(PIN_MOTOR_RIGHT_FORWARD,LOW);  
  }
  else {
    analogWrite(PIN_MOTOR_RIGHT_FORWARD,right);
    digitalWrite(PIN_MOTOR_RIGHT_REVERSE,LOW);
  }

  if (left < 0) {
    analogWrite(PIN_MOTOR_LEFT_REVERSE,abs(left));
    digitalWrite(PIN_MOTOR_LEFT_FORWARD,LOW);
  }
  else {
    analogWrite(PIN_MOTOR_LEFT_FORWARD,left);
    digitalWrite(PIN_MOTOR_LEFT_REVERSE,LOW); 
  }
}

////////////////////////////////////////////////////////////////////////////////

#ifdef DEBUG
void debug() {

  for (int i = 0; i < (SEN_NUM_SENSORS-1); i++) {
    Serial.print(g_sensors_array[i]);
    Serial.print("\t");
  }
  Serial.print(g_sensors_array[SEN_NUM_SENSORS-1]);
  Serial.print(" ==> ");
  Serial.print(g_newdirection);
  Serial.print(" (lastturn: ");
  Serial.print(g_lastturn);
  if(g_tracklost) {
    unsigned int speedcorrtracklost = (int)(g_tracklost*SPEED_MAX*0.1);
    if(speedcorrtracklost < TRACK_LOST_TIMEOUT) {
      // We consider this a temporary tracklost: keep going the same direction
      Serial.print(" temporary tracklost: "); 
    }
    else if(speedcorrtracklost >= TRACK_LOST_TIMEOUT &&
            speedcorrtracklost < COMPLETELY_LOST_TIMEOUT ) {
      // If we've been lost long enough, try to find back on track
      // What was the last recorded turn? Take the same turn again
      Serial.print(" longer tracklost: "); 
    } 
    else if(speedcorrtracklost >= COMPLETELY_LOST_TIMEOUT) {
      // Stop if we are completely lost
      Serial.print(" completely lost, STOP!: ");   
    }
    Serial.print(g_tracklost);
  }
  Serial.println(")");
}
#endif

////////////////////////////////////////////////////////////////////////////////