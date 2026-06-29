#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// 모터 핀 설정
#define IN1_PIN PORTD2
#define IN2_PIN PORTD3

// 초음파 센서 핀 설정 (D6, D7)
#define TRIG_PIN PORTD6
#define ECHO_PIN PORTD7


// 1. USART 통신 관련 함수 (9600bps)
void USART_Init(void) {
	UBRR0H = 0;
	UBRR0L = 103; // 16MHz 기준 9600bps
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8비트 데이터
	UCSR0B = (1 << TXEN0) | (1 << RXEN0); // 송신(TX) 및 수신(RX) 활성화
}

void USART_transmit(char data) {
	while (!(UCSR0A & (1 << UDRE0))); // 버퍼 대기
	UDR0 = data;
}

void USART_transmit_string(const char* str) {
	while (*str) {
		USART_transmit(*str++);
	}
}

void USART_transmit_number(uint16_t num) {
	char buffer[10];
	int i = 0;

	if (num == 0) {
		USART_transmit('0');
		return;
	}

	while (num > 0) {
		buffer[i++] = (num % 10) + '0';
		num /= 10;
	}

	for (int j = i - 1; j >= 0; j--) {
		USART_transmit(buffer[j]);
	}
}

// 수신 버퍼 확인 후 데이터 읽기 (비동기)
uint8_t USART_receive(void) {
	if (UCSR0A & (1 << RXC0)) {
		return UDR0;
	}
	return 0; // 데이터 없으면 0 반환
}

// 2. 초음파 센서 (HC-SR04) 제어 함수
uint16_t get_distance_cm(void) {
	uint32_t timeout;

	// 이전 측정 잔여 신호 대기
	timeout = 30000;
	while (PIND & (1 << ECHO_PIN)) {
		if (--timeout == 0) break;
	}

	// Trig 10us HIGH 출력
	PORTD &= ~(1 << TRIG_PIN);
	_delay_us(2);
	PORTD |= (1 << TRIG_PIN);
	_delay_us(10);
	PORTD &= ~(1 << TRIG_PIN);

	// Echo HIGH 대기
	timeout = 50000;
	while (!(PIND & (1 << ECHO_PIN))) {
		if (--timeout == 0) return 999; // 센서 응답 없음
	}

	// Echo 유지 시간 측정
	uint32_t echo_time = 0;
	while (PIND & (1 << ECHO_PIN)) {
		echo_time++;
		_delay_us(1);
		
		// 무한 루프 방지
		if (echo_time > 35000) return 888; // 측정 거리 한계 초과
	}

	// 거리 계산 (오버헤드 보정: 42)
	return (uint16_t)(echo_time / 42);
}


// 3. 타이머 및 모터 제어 함수
void timer1_init(void) {
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

	ICR1 = 1999;
	OCR1A = 0;
}

void motor_drive(uint8_t dir, uint16_t pwm) {
	if (dir == 0) {
		PORTD |=  (1 << IN1_PIN);  // 정방향
		PORTD &= ~(1 << IN2_PIN);
		} else {
		PORTD &= ~(1 << IN1_PIN);  // 역방향
		PORTD |=  (1 << IN2_PIN);
	}
	OCR1A = pwm;
}

void motor_brake(void) {
	PORTD |= (1 << IN1_PIN);
	PORTD |= (1 << IN2_PIN);
	_delay_ms(1000); // 확실한 제동 딜레이
	OCR1A = 0;
}


// 4. 메인 루프
int main(void) {
	DDRB |= (1 << PORTB1); // D9, ENA
	DDRD |= (1 << IN1_PIN) | (1 << IN2_PIN) | (1 << TRIG_PIN);
	DDRD &= ~(1 << ECHO_PIN);

	timer1_init();
	USART_Init();

	uint8_t motor_dir = 0; // 0: 정방향
	uint16_t print_counter = 0;

	while (1) {
		// --- 시리얼 수신에 따른 모터 구동 ---
		uint8_t rx_data = USART_receive();
		
		if (rx_data == '1') {
			motor_drive(motor_dir, 1999); // 최고 속도 회전
			} else if (rx_data == '0') {
			OCR1A = 0; // 물방울 감지 안 되면 부드럽게 정지 (PWM 0)
		}

		// --- 초음파 센서 장애물 감지 및 출력 ---
		print_counter++;
		
		// 잔향 방지를 위한 측정 주기 딜레이
		if (print_counter >= 60000) {
			uint16_t dist = get_distance_cm();
			
			USART_transmit_string("Dist: ");
			USART_transmit_number(dist);
			USART_transmit_string(" cm\r\n");
			
			// 장애물 근접 시 즉시 브레이크
			if(dist < 10 && dist > 0) {
				USART_transmit_string("OBSTACLE DETECTED! BRAKE!\r\n");
				motor_brake();
				motor_dir = !motor_dir; // 충돌 후 역방향 전환 - 방향 제어 연습용
			}
			print_counter = 0;
		}
	}
}