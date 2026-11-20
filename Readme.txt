GPIO	Назначение	Куда подключить
GP0	Serial1 TX	→ RX сенсора отпечатков
GP1	Serial1 RX	← TX сенсора отпечатков
GP2	DIN кольца	→ DIN кольца WS2812 (8 шт)
GP3	Реле	→ IN модуля реле
GP4	Buzzer	→ + buzzer
GP5	Кнопка ENROLL	между GP5 и GND
GP6	Кнопка DELETE	между GP6 и GND
Питание: сенсор/кольцо/реле по VCC(3.3–5 В) и GND. Кнопки — на GND (внутренняя pull-up).