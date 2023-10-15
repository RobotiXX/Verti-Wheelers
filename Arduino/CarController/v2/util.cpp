#include "CarController.h"
#include "def.h"

bool prev_dir = 0;
unsigned long brk_timer = 0;

float fmap(float toMap, float in_min, float in_max, float out_min, float out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float fw_to_rev(float th){
  if (brk_timer == 0){
    brk_timer = millis();
  }

  if((millis() - brk_timer) < brk_delay ){
    return th;
  }
  else{

    if((millis() - brk_timer) < (brk_delay + 200)){
      return 0.0;
    }
    else{
      brk_timer = 0;
      prev_dir = 1;
      return th;
    }

  }
}

float rev_to_fw(){
  if (brk_timer == 0){
    brk_timer = millis();
  }

  if((millis() - brk_timer) < (brk_delay + 100) ){
    return 0.0;
  }
  else{
      brk_timer = 0;
      prev_dir = 0;
      return 0.0;
  }

}
