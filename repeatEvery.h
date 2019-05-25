
void repeatEvery (unsigned long repeatTime, float ff(bool, unsigned long&)){
  unsigned long lastTime;
  float v1= ff(0, lastTime); // execute the function, 0 flag
     // 0 flag writes the current value of ff's local lastTime into the parameter variable
  if (millis()-lastTime>=repeatTime){ // time has elapsed
      while (millis()-lastTime>=repeatTime) lastTime += repeatTime; // update latest execution time
      v1= ff(1,lastTime); // execute the function, 1 flag
     // 1 flag updates ff's local lastTime to the new value
  }
}

void repeatEvery (unsigned long repeatTime, unsigned long ff(bool, unsigned long&)){
  unsigned long lastTime;
  unsigned long v1= ff(0, lastTime); // execute the function, 0 flag
     // 0 flag writes the current value of ff's local lastTime into the parameter variable
  if (millis()-lastTime>=repeatTime){ // time has elapsed
      while (millis()-lastTime>=repeatTime) lastTime += repeatTime; // update latest execution time
      v1= ff(1,lastTime); // execute the function, 1 flag
     // 1 flag updates ff's local lastTime to the new value
  }
}

void repeatEvery (unsigned long repeatTime, void ff(bool, unsigned long&)){
  unsigned long lastTime;
  ff(0, lastTime); // execute the function, 0 flag
     // 0 flag writes the current value of ff's local lastTime into the parameter variable
  if (millis()-lastTime>=repeatTime){ // time has elapsed
      while (millis()-lastTime>=repeatTime) lastTime += repeatTime; // update latest execution time
      ff(1,lastTime); // execute the function, 1 flag
     // 1 flag updates ff's local lastTime to the new value
  }
}


