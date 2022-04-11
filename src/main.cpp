/*Передатчик 433 МГц (от кнопки (геркон)-удалено) 
 * работает по прерыванию в цикле от PIT
 * включил внутренний генератор 32КГц на 1КГц и 
 * добавил счетчик myT чтобы регулировать период.Датчик BME280. 
 * Опознавание датчика нет пока
*/
#define G433_SPEED 2000   // скорость 100-8000 бит/с, по умолч. 2000 бит/с
#define HERCON PIN_PA1   // Нога геркона
#define RADIO_VCC PIN_PA7 // запитка передатчика
#define ledPin PIN_PA6 //мигалка для визуализации
#define BME_VCC PIN_PA5 //запитка BME280  
#include <Arduino.h>
#include <Gyver433.h>
#include <SimpleBME280.h>    // Подключение библиотеки
#include <util/delay.h>

Gyver433_TX <PIN_PA4, G433_XOR> tx;  // указали пин и размер буфера
SimpleBME280 bme280;
 float data[4];
 //unsigned int DutyCycle = 0;
 uint8_t myT = 0;
 
void setup() {
 //   Serial.begin(115200);

    if (bme280.begin() == 0)  
//      Serial.print(F("BME280 initialized.\n"));
    
    pinMode(ledPin, OUTPUT); 
    pinMode(HERCON, INPUT);  //Геркон на +
    pinMode(RADIO_VCC, OUTPUT);
    pinMode(BME_VCC, OUTPUT);      

//    digitalWrite(ledPin,HIGH);
    SLPCTRL.CTRLA = 0b00000101;
    PORTA.PIN1CTRL =0b00000101; //Low level-5, bothedges -1
    sei();

  CLKCTRL.OSC32KCTRLA = 0b010; //Разрешаем генератору 32К работу в режиме сна
 // RTC.CTRLA = 0b01110001;
  RTC.CLKSEL = 0b01; //Выбираем генератор 32Кгц, если 0b01 то 1Кгц
  RTC.PITCTRLA = 0b01100001; //Выбираем период между прерываниями - CYC16384 (цикла)
  RTC.PITINTCTRL = 0b01; //разрешаем периодические прерывания
}

void loop() 
{
  myT++ ;
  if (myT == 2 ) {
  digitalWrite(BME_VCC, HIGH);  //Запитываем датчик
  digitalWrite(ledPin,HIGH);
  delay(500);     //Надо уточнить и по возможности уменьшить задержку
  myT = 0;
  data[0] = 0xA9;
  bme280.update();
  data[1] = bme280.getT();
  data[2] = bme280.getP();
  data[3] = bme280.getH();

 /* p = p/133.32; перевод Па в мм.рт.ст
  data[1] = t;
  data[2] = p;
  data[3] = h;
 */

  delay(1000);    //????????


  digitalWrite(ledPin, HIGH);  //test
  _delay_ms(1);
  tx.sendData(data);
  digitalWrite(RADIO_VCC, LOW);
  digitalWrite(ledPin, LOW);  //test

  digitalWrite(BME_VCC, LOW);  // Все выключаем и спим
//  power.sleepDelay(30000);    // Спим 30 сек

 }
  
  asm("sleep");
}
// Прерывание для геркона
ISR(PORTA_PORT_vect)  //Обработчик прерывания порт А
{
  _delay_ms(5);
  digitalWrite(RADIO_VCC,HIGH);  //включаем передатчик
  _delay_ms(5);
  data[0] = 0xA1;
  tx.sendData(data);
  digitalWrite(RADIO_VCC,LOW);  //и отключаем передатчик для экономии энергии
  PORTA.INTFLAGS = 0b00000010;  //Сбрасываем флаг, прерывание обработано
   } 
//
ISR(RTC_PIT_vect) {       //Этот обработчик прерывания снимает флаг прерывания по PIT
  RTC.PITINTFLAGS = 0b01;
}