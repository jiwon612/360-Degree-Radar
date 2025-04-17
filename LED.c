 // slave 1 : led
 #include <avr/io.h>
 #include <avr/interrupt.h>
 volatile uint8_t distance1 = 0;
 volatile uint8_t distance2 = 0;
 volatile uint8_t distance3 = 0;
 volatile uint8_t currentDistance = 0;
 void setup() {
 // SPI 설정
SPI_init();
 // 3, 5, 6번 핀을 출력으로 설정
DDRD |= (1 << DDD3) | (1 << DDD5) | (1 << DDD6);
// Timer2 설정 (3번 핀, OCR2B 제어)
 TCCR2A |= (1 << WGM20) | (1 << WGM21) | (1 << COM2B1); // 고속 PWM
모드, 비반전
TCCR2B |= (1 << CS21); // 분주비 8
 // Timer0 설정 (5번 핀과 6번 핀, OCR0A와 OCR0B 제어)
 TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1) | (1 <<
 COM0B1); // 고속 PWM 모드, 비반전
TCCR0B |= (1 << CS01); // 분주비 8
 }
 void loop() {
 // 거리 값이 작을수록 밝아지도록 0~255 범위의 PWM 값 계산
int brightness1 = 255- (distance1 * 255 / 40);
 int brightness2 = 255- (distance2 * 255 / 40);
 int brightness3 = 255- (distance3 * 255 / 40);
 // 3번 핀의 밝기 설정 (Timer2, OCR2B)
 OCR2B = brightness1;
 // 5번 핀의 밝기 설정 (Timer0, OCR0B)
 OCR0B = brightness2;
 // 6번 핀의 밝기 설정 (Timer0, OCR0A)
 OCR0A = brightness3;
 }
 void SPI_init() {
 DDRB &= ~(1 << DDB4); // MISO를 제외한 SPI 핀을 슬레이브로 설정
SPCR = (1 << SPE) | (1 << SPIE); // SPI 설정, 슬레이브 모드 및 인터럽트
활성화
sei(); // 전역 인터럽트 활성화
}
 ISR(SPI_STC_vect) {
 static bool identifierReceived = false;
 if (!identifierReceived) {
 // 첫 번째 바이트 수신: 식별자
currentDistance = SPDR;
 identifierReceived = true;
 } else {
 // 두 번째 바이트 수신: 거리 값
if (currentDistance == 1) distance1 = SPDR;
 else if (currentDistance == 2) distance2 = SPDR;
 else if (currentDistance == 3) distance3 = SPDR;
 identifierReceived = false;
 }
 }
