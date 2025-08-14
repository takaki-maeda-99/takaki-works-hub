#include <LiquidCrystal.h>

const int rs = 16, en = 17, d4 = 15, d5 = 19, d6 = 20, d7 = 21;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// 割り込み内で使うな！
void lcdPrintLine(uint8_t row, const char* fmt, ...) {
    char buf[17];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    lcd.setCursor(0, row);
    lcd.print(buf);
}