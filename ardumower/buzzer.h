#ifndef BUZZER_H
#define BUZZER_H

class BuzzerClass {
    public:
      void begin();      
      void tone(unsigned int freq);
      void noTone(); 
    protected:     
  
};

extern BuzzerClass Buzzer;

#endif

