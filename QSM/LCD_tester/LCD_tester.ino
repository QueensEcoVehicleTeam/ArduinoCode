#include <font_Arial.h>
#include <font_ArialBold.h>
#include <ILI9341_t3.h>


#define TFT_cs 10
#define TFT_dc 9
#define TFT_reset 8

TFT QSM_LCD = TFT(TFT_cs, TFT_dc, TFT_reset);

void setup()
{
  QSM_LCD.begin();
  QSM_LCD.background(255,255, 255);
  QSM_LCD.stroke(255, 255, 255);
  QSM_LCD.setTextSize(2);
  QSM_LCD.text("Eddy is a lil bitch", 0, 0);
}

void loop()
{
  delay(100);
}
