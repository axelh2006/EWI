const uint8_t no_note = 255;

const uint8_t key_to_number[32] = {
  28, 24, 26, 22, 27, 23, 25, no_note,

  21, 17, 19, 15, 20, 16, 18, no_note,

  7, 3, 5, 1, 6, 2, 4, no_note,

  14, 10, 12, 8, 13, 9, 11, no_note
};

const uint8_t key_to_note_wickihaydn[32] = {
    0, 
    44, 46, 48, 50, 52, 54, 56,
    49, 51, 53, 55, 57, 59, 61,
    56, 58, 60, 62, 64, 66, 68,
    61, 63, 65, 67, 69, 71, 73
};