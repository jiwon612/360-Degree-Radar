// servo_control_to_stm
 #define FOSC 16000000UL // CPU 클럭 속도
#define BAUD 9600
 #define MYUBRR FOSC/16/BAUD-1
 // USART와 SPI 초기화, 데이터 전송 함수 선언
void USART_Init(unsigned int ubrr);
 void USART_Transmit(unsigned char data);
 void USART_Transmit_String(const char* str);
 void send_to_STM(int data);
 void SPI_init();
 // 전역 변수 선언
volatile uint8_t distance1 = 0; // 첫 번째 거리 데이터
volatile uint8_t distance2 = 0; // 두 번째 거리 데이터
volatile uint8_t distance3 = 0; // 세 번째 거리 데이터
volatile uint8_t currentDistance = 0; // 현재 수신 중인 거리 데이터
int sum1 = 0;
 // 거리 합산을 위한 변수
int count1 = 0; // 유효한 거리 데이터의 개수
static bool inStream1 = false; // 데이터 스트림 상태
float first_angle1 = 0; // 첫 각도 저장
int sum2 = 0;
 // 거리 합산을 위한 변수
int count2 = 0; // 유효한 거리 데이터의 개수
static bool inStream2 = false; // 데이터 스트림 상태
float first_angle2 = 0; // 첫 각도 저장
int sum3 = 0;
 // 거리 합산을 위한 변수
int count3 = 0; // 유효한 거리 데이터의 개수
static bool inStream3 = false; // 데이터 스트림 상태
float first_angle3 = 0; // 첫 각도 저장
// 거리 및 각도 정보를 저장할 구조체
typedef struct {
 int servo_num; // 해당 서보 번호
int dis_average; // 평균 거리
float ang_average; // 평균 각도
} Detected_data;
 Detected_data d1 = {1, 255, 0.0f}; // 거리 및 각도 평균 정보를 담는 구조체
변수
Detected_data* d1_ptr = &d1;
 Detected_data d2 = {2, 255, 0.0f}; // 거리 및 각도 평균 정보를 담는 구조체
변수
Detected_data* d2_ptr = &d2;
 Detected_data d3 = {3, 255, 0.0f}; // 거리 및 각도 평균 정보를 담는 구조체
변수
Detected_data* d3_ptr = &d3;
 // 초기 설정 함수
void setup() {
 DDRB |= 0x01; // PB0 핀을 출력으로 설정 (TX)
 USART_Init(MYUBRR); // UART 초기화
SPI_init(); // SPI 초기화
// 서보 모터 제어 핀을 출력으로 설정
DDRB |= (1 << DDB1); // 핀 9 출력 설정
// Timer1 설정: PWM, Phase Correct 모드
TCCR1A = 0;
 TCCR1B = 0;
 ICR1 = 19999; // 50Hz (20ms 주기) 설정
TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // Non-inverting 모드
TCCR1A |= (1 << WGM11); // PWM, Phase Correct 모드 설정
TCCR1B |= (1 << WGM13) | (1 << WGM12); // PWM 모드 선택
TCCR1B |= (1 << CS11); // 프리스케일러 8 설정
}
 // 메인 반복 함수
void loop() {
 // 서보가 0도에서 180도까지 전진하며 회전
for (float angle = 0; angle <= 180; angle++) {
 OCR1A = 1000 + (angle * (5000- 1000)) / 180; // 각도에 따라 PWM
신호를 설정
cal_distance_average(d1_ptr, d2_ptr, d3_ptr, distance1,
 distance2, distance3, angle); // 거리 및 각도 평균 계산
delay(50); // 부드러운 움직임을 위한 짧은 지연
send_to_STM(d1_ptr->servo_num);
 send_to_STM(d1_ptr->dis_average);
 send_to_STM((int)d1_ptr->ang_average);
 send_to_STM(d2_ptr->servo_num);
 send_to_STM(d2_ptr->dis_average);
 send_to_STM((int)d2_ptr->ang_average);
 send_to_STM(d3_ptr->servo_num);
 send_to_STM(d3_ptr->dis_average);
 send_to_STM((int)d3_ptr->ang_average);
 // Serial.print("\n");
 d1 = {1, 255, 0.0f};
 d2 = {2, 255, 0.0f};
 d3 = {3, 255, 0.0f};
 // 서보가 180도에서 0도까지 후진하며 회전
for (float angle = 180; angle >= 0; angle--) {
 OCR1A = 1000 + (angle * (5000- 1000)) / 180; // 각도에 따라 PWM
신호를 설정
cal_distance_average(d1_ptr, d2_ptr, d3_ptr, distance1,
 distance2, distance3, angle); // 거리 및 각도 평균 계산
delay(50); // 부드러운 움직임을 위한 짧은 지연
send_to_STM(d1_ptr->servo_num);
 send_to_STM(d1_ptr->dis_average);
 send_to_STM((int)d1_ptr->ang_average);
 send_to_STM(d2_ptr->servo_num);
 send_to_STM(d2_ptr->dis_average);
 send_to_STM((int)d2_ptr->ang_average);
 send_to_STM(d3_ptr->servo_num);
 send_to_STM(d3_ptr->dis_average);
 send_to_STM((int)d3_ptr->ang_average);
 d1 = {1, 255, 0.0f};
 d2 = {2, 255, 0.0f};
 d3 = {3, 255, 0.0f};
 }
// USART 초기화 함수
void USART_Init(unsigned int ubrr) {
 UBRR0H = (unsigned char)(ubrr >> 8); // UBRR 레지스터 설정 (상위
바이트)
 UBRR0L = (unsigned char)ubrr;
바이트)
 UCSR0B = (1 << RXEN0) | (1 << TXEN0); // 수신 및 송신 활성화
UCSR0C = (1 << UCSZ01) | (3 << UCSZ00); // 데이터 형식: 8비트, 1 스톱
비트
}
 // UBRR 레지스터 설정 (하위
// 한 글자 송신 함수
void USART_Transmit(unsigned char data) {
 while (!(UCSR0A & (1 << UDRE0))); // 송신 가능 대기
UDR0 = data; // 데이터 전송
}
 // 문자열 전송 함수
void USART_Transmit_String(const char* str) {
 while (*str) { // 문자열 끝까지 반복
USART_Transmit(*str++);
 }
 }
 // STM 보드로 데이터 전송 함수
void send_to_STM(int data) {
 char buffer[10];
 itoa(data, buffer, 10); // 데이터를 문자열로 변환
USART_Transmit_String(buffer); // 문자열 전송
USART_Transmit('\0'); // 문자열 종료 문자 전송
}
 // 거리와 각도의 평균을 계산하는 함수
void cal_distance_average(Detected_data *d1, Detected_data *d2,
 Detected_data *d3, const int distance1, const int distance2, const
 int distance3, const float angle) {
 // // distance1 데이터가 0일 경우
if (distance1 > 200) {
 if (inStream1) {
 if (count1 > 0) { // 평균값 계산
if (d1->dis_average > (sum1 / count1)) {
 d1->dis_average = sum1 / count1;
 d1->ang_average = (first_angle1 + angle) / 2;
 d1->servo_num = 1;
 }
 sum1 = 0;
 count1 = 0;
 inStream1 = false;
 first_angle1 = 0;
}
 }
 } else {
 first_angle1 = inStream1 ? first_angle1 : angle;
 inStream1 = true;
 sum1 += distance1;
 count1++;
 }
 // distance2 데이터가 0일 경우
if (distance2 > 200) {
 if(inStream2) {
 if (count2 > 0) {
 if (d2->dis_average > (sum2 / count2)) {
 d2->dis_average = sum2 / count2;
 d2->ang_average = (first_angle2 + angle) / 2;
 d2->servo_num = 2;
 }
 sum2 = 0;
 count2 = 0;
 inStream2 = false;
 first_angle2 = 0;
 }
 }
 }
 else {
 first_angle2 = inStream2 ? first_angle2 : angle;
 inStream2 = true;
 sum2 += distance2;
 count2++;
 }
 // distance3 데이터가 0일 경우
if (distance3 > 200) {
 if(inStream3) {
 if (count3 > 0) {
 if (d3->dis_average > (sum3 / count3)) {
 d3->dis_average = sum3 / count3;
 d3->ang_average = (first_angle3 + angle) / 2;
 d3->servo_num = 3;
 }
 sum3 = 0;
 count3 = 0;
 inStream3 = false;
 first_angle3 = 0;
 }
 }
 }
 else {
 first_angle3 = inStream3 ? first_angle3 : angle;
inStream3 = true;
 sum3 += distance3;
 count3++;
 }
 }
 // SPI 초기화 함수
void SPI_init() {
 DDRB &= ~(1 << DDB4); // MISO 핀을 제외한 SPI 핀을 슬레이브로 설정
SPCR = (1 << SPE) | (1 << SPIE); // SPI 활성화 및 인터럽트 활성화
sei(); // 전역 인터럽트 활성화
}
 // SPI 통신 인터럽트 서비스 루틴
ISR(SPI_STC_vect) {
 static bool identifierReceived = false; // 수신 식별 상태 플래그
if (!identifierReceived) { // 첫 번째 바이트 수신 시 식별자 처리
currentDistance = SPDR; // 수신된 데이터 저장
identifierReceived = true;
 }
 else { // 두 번째 바이트 수신 시 거리 값 업데이트
if (currentDistance == 1) distance1 = SPDR;
 else if (currentDistance == 2) distance2 = SPDR;
 else if (currentDistance == 3) distance3 = SPDR;
 identifierReceived = false; // 다음 데이터 수신 준비
}
 }
