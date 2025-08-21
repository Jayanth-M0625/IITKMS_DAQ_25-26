#include <LCDWIKI_KBV.h>

LCDWIKI_KBV tft(ILI9486, A3, A2, A1, A0, A4); // Verify driver IC

void setup() {
  tft.Init_LCD();
  tft.Set_Rotation(1);
  tft.Fill_Screen(0x0000); // Black background
  
  // Display centered text
  printCenteredText("It Works!", 100);
  printCenteredText("IITK Motorsports", 150);
}

void printCenteredText(String text, int y) {
  tft.Set_Text_Size(3);
  int char_width = 18; // Approx. pixels per char at size 3 (5px * 3.6 scale)
  int x = (tft.Get_Width() - (text.length() * char_width)) / 2;
  tft.Print_String(text, x, y); // Use library's Print_String method
}

void loop() {}
