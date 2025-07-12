// Define the pins for each segment (a, b, c, d, e, f, g) and the common cathode pins
const int segmentPins[] = {2, 3, 4, 5, 6, 7, 8};
const int digitPins[] = {9, 10, 11, 12}; // Common cathode pins

// Define the digit patterns for digits 0-9
const byte digitPatterns[] = {
  B00111111, // 0
  B00000110, // 1
  B01011011, // 2
  B01001111, // 3
  B01100110, // 4
  B01101101, // 5
  B01111101, // 6
  B00000111, // 7
  B01111111, // 8
  B01101111  // 9
};

void setup() {
  // Set the digitPins as OUTPUT
  for (int i = 0; i < 4; i++) {
    pinMode(digitPins[i], OUTPUT);
  }
  // Set the segmentPins as OUTPUT
  for (int i = 0; i < 7; i++) {
    pinMode(segmentPins[i], OUTPUT);
  }
}

void loop() {
  // Display digits 0-9 in a loop
  for (int i = 0; i < 10; i++) {
    displayDigit(i);
    delay(1000); // Display each digit for 1 second
  }
}

void displayDigit(int digit) {
  if (digit >= 0 && digit <= 9) {
    // Select the digit
    for (int i = 0; i < 4; i++) {
      if (i == digit) {
        digitalWrite(digitPins[i], LOW); // Common cathode display, so set LOW to enable the digit
      } else {
        digitalWrite(digitPins[i], HIGH);
      }
    }

    // Display the segments for the selected digit
    for (int i = 0; i < 7; i++) {
      if (bitRead(digitPatterns[digit], i)) {
        digitalWrite(segmentPins[i], HIGH); // Turn on the segment
      } else {
        digitalWrite(segmentPins[i], LOW); // Turn off the segment
      }
    }
  }
}
