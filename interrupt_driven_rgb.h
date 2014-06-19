enum Base {
  TOP,
  CENTER,
  BOTTOM
};

#define NOTE_R0 0

typedef struct Note {
  unsigned int tone;
  unsigned int duration;
} Note;

Note song[] = {
  {NOTE_C4, 4},
  {NOTE_D4, 4},
  {NOTE_E4, 4},
};
