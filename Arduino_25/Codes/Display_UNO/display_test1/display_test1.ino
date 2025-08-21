
#include <LCDWIKI_GUI.h> // Core graphics library
#include <LCDWIKI_KBV.h> // Hardware-specific library



// For 3.5" UNO shield (ILI9486):
LCDWIKI_KBV tft(ILI9486, A3, A2, A1, A0, A4); // Model, CS, CD, WR, RD, RST [1][4]


//void setup() {
//  tft.Init_LCD();
//  tft.Fill_Screen(0x0000); // Black background
//  tft.Set_Text_colour(0x07E0); // Green text
//  tft.Print_String("System Ready", 100, 160);
//}
//
//void loop() {
//  // Test pattern
//  tft.Set_Draw_color(0xF800); // Red
//  tft.Draw_Circle(160, 120, 50); // X,Y,Radius
//  delay(1000);
//  tft.Fill_Circle(160, 120, 50);
//  delay(1000);
//}

void setup() {
  tft.Init_LCD();
  tft.Set_Rotation(1); // Portrait orientation
  tft.Fill_Screen(0x0000); // Black background
  
  tft.Set_Text_Size(3);
  tft.Set_Text_colour(0x07E0); // Green text
  tft.Print_String("Success!", 80, 160);
}
//// Rotating triangle demo
//void loop() {
//  static uint16_t angle = 0;
//  int16_t center_x = 160; // Screen center X
//  int16_t center_y = 120; // Screen center Y
//  int16_t radius = 50;    // Triangle size
//  
//  // Calculate triangle vertices using trigonometry
//  tft.Set_Draw_color(0xF800); // Red
//  tft.Draw_Triangle(
//    center_x + radius * cos(radians(angle)),          // Vertex 1 X
//    center_y + radius * sin(radians(angle)),          // Vertex 1 Y
//    center_x + radius * cos(radians(angle + 120)),    // Vertex 2 X
//    center_y + radius * sin(radians(angle + 120)),    // Vertex 2 Y
//    center_x + radius * cos(radians(angle + 240)),    // Vertex 3 X
//    center_y + radius * sin(radians(angle + 240))     // Vertex 3 Y
//  );
//  
//  angle += 5;
//  delay(100);
//  tft.Fill_Screen(0x0000); // Clear screen each frame
//}
//
//void loop() {
//  static uint16_t angle = 0;
//  int16_t center_x = 160; // Screen center X
//  int16_t center_y = 120; // Screen center Y
//  int16_t radius = 50;    // Triangle size
//  
//  // Calculate triangle vertices using trigonometry
//  tft.Set_Draw_color(0x0000); // Black (darker color)
//  
//  // Draw thicker lines by repeating the draw operation
//  for (int thickness = 1; thickness <= 3; thickness++) {
//    tft.Draw_Triangle(
//      center_x + radius * cos(radians(angle)) - thickness, 
//      center_y + radius * sin(radians(angle)) - thickness,
//      center_x + radius * cos(radians(angle + 120)) - thickness, 
//      center_y + radius * sin(radians(angle + 120)) - thickness,
//      center_x + radius * cos(radians(angle + 240)) - thickness, 
//      center_y + radius * sin(radians(angle + 240)) - thickness
//    );
//    
//    tft.Draw_Triangle(
//      center_x + radius * cos(radians(angle)) + thickness, 
//      center_y + radius * sin(radians(angle)) + thickness,
//      center_x + radius * cos(radians(angle + 120)) + thickness, 
//      center_y + radius * sin(radians(angle + 120)) + thickness,
//      center_x + radius * cos(radians(angle + 240)) + thickness, 
//      center_y + radius * sin(radians(angle + 240)) + thickness
//    );
//    
//    tft.Draw_Triangle(
//      center_x + radius * cos(radians(angle)) - thickness, 
//      center_y + radius * sin(radians(angle)) + thickness,
//      center_x + radius * cos(radians(angle + 120)) - thickness, 
//      center_y + radius * sin(radians(angle + 120)) + thickness,
//      center_x + radius * cos(radians(angle + 240)) - thickness, 
//      center_y + radius * sin(radians(angle + 240)) + thickness
//    );
//    
//    tft.Draw_Triangle(
//      center_x + radius * cos(radians(angle)) + thickness, 
//      center_y + radius * sin(radians(angle)) - thickness,
//      center_x + radius * cos(radians(angle + 120)) + thickness, 
//      center_y + radius * sin(radians(angle + 120)) - thickness,
//      center_x + radius * cos(radians(angle + 240)) + thickness, 
//      center_y + radius * sin(radians(angle + 240)) - thickness
//    );
//  }
//  
//  angle += 5;
//  delay(100);
//  tft.Fill_Screen(0x0000); // Clear screen each frame
//}
