// master
 #define TRIG1 2
 #define ECHO1 3
 #define TRIG2 4
 // 초음파 센서 1 Trig 핀
// 초음파 센서 1 Echo 핀
// 초음파 센서 2 Trig 핀
#define ECHO2 5
 #define TRIG3 6
 // 초음파 센서 2 Echo 핀
// 초음파 센서 3 Trig 핀
#define ECHO3 7
 // 초음파 센서 3 Echo 핀
#define F_CPU 16000000UL // 16MHz 아두이노 클럭 속도
#define CS1_PIN PB1 // Chip Select for slave1
 #define CS2_PIN PB2 // Chip Select for slave2
 const int DISTANCE_OFFSET1 =-39;
 const int DISTANCE_OFFSET2 =-39;
 const int DISTANCE_OFFSET3 =-10;
 long distance1, distance2, distance3;
 unsigned long lastTrigTime = 0;
 unsigned long lastPrintTime = 0;
 void setup()
 {
 // SPI Master 설정
SPI_init();
 // Chip Select pins as output
 DDRB |= (1 << CS1_PIN) | (1 << CS2_PIN);
 // Set default CS pins to HIGH
 PORTB |= (1 << CS1_PIN) | (1 << CS2_PIN);
 // 핀 설정
DDRD |= (1 << TRIG1) | (1 << TRIG2) | (1 << TRIG3);
 DDRD &= ~((1 << ECHO1) | (1 << ECHO2) | (1 << ECHO3));
 TCCR1A = 0;
 TCCR1B = (1 << CS10); // Timer1 prescaler 설정
}
 void loop()
 {
 unsigned long currentMillis = millis();
 if (currentMillis- lastTrigTime >= 60) {
 lastTrigTime = currentMillis;
 int measured1 = measureDistance(TRIG1, ECHO1);
 int measured2 = measureDistance(TRIG2, ECHO2);
 int measured3 = measureDistance(TRIG3, ECHO3);
 distance1 = (measured1 == 255) ? 255 : (measured1 +
 DISTANCE_OFFSET1);
 distance2 = (measured2 == 255) ? 255 : (measured2 +
 DISTANCE_OFFSET2);
 distance3 = (measured3 == 255) ? 255 : (measured3 +
 DISTANCE_OFFSET3);
}
 if (currentMillis- lastPrintTime >= 1000) {
 lastPrintTime = currentMillis;
 }
 // SPI로 distance1,2,3 전송 to both slaves
 if (distance1 < 100) {
 transmitToSlave(CS2_PIN, 1, distance1);
 transmitToSlave(CS2_PIN, 2, distance2);
 transmitToSlave(CS2_PIN, 3, distance3);
 transmitToSlave(CS1_PIN, 1, distance1);
 transmitToSlave(CS1_PIN, 2, distance2);
 transmitToSlave(CS1_PIN, 3, distance3);
 }
 // Timer 2를 활용한 10µs 대기 함수
void delayMicroseconds10()
 {
 TCCR2A = 0;
 TCCR2B = (1 << CS20); // 분주율 1로 설정 (62.5ns 해상도)
 TCNT2 = 0; // Timer 2 카운터 초기화
while (TCNT2 < 160); // 160 카운트 대기 (10µs)
 TCCR2B = 0; // Timer 2 정지
}
 uint8_t measureDistance(int trigPin, int echoPin)
 {
 PORTD |= (1 << trigPin);
 delayMicroseconds10();
 PORTD &= ~(1 << trigPin);
 TCNT1 = 0;
 TCCR1B |= (1 << CS10);
 while (!(PIND & (1 << echoPin)) && TCNT1 < 65000);
 while ((PIND & (1 << echoPin)) && TCNT1 < 65000);
 TCCR1B = 0;
 if (TCNT1 >= 65000) {
 return 255;
 } else {
 int duration = TCNT1 * 0.0625;
 return (int)(duration * 0.034) / 2;
 }
 }
 void SPI_init()
{
 DDRB |= (1 << DDB2) | (1 << DDB3) | (1 << DDB5);
 SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
 }
 void transmitToSlave(uint8_t cs_pin, uint8_t identifier, uint8_t
 distanceValue)
 {
 PORTB &= ~(1 << cs_pin); // Select slave by setting CS low
 SPDR = identifier;
 while (!(SPSR & (1 << SPIF))); // Wait for transmission to complete
 SPDR = distanceValue;
 while (!(SPSR & (1 << SPIF))); // Wait for transmission to complete
 PORTB |= (1 << cs_pin); // Deselect slave by setting CS high
 }
