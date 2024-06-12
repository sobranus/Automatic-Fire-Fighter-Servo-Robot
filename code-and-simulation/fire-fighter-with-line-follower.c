#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define F_CPU 16000000UL

// batas deteksi sensor (sesuaikan dengan kalibrasi sensor)
#define FIRE_THRESHOLD 500 
#define LINE_THRESHOLD 500

// sudut servo
#define DEGREE_180 125	//125 = 2 ms = 180 derajat
#define DEGREE_90 94	//93.8 = 1.5 ms = 90 derajat
#define DEGREE_0 62	//62.5 = 1 ms = 0 derajat

//deklarasi variabel
int PULSE_WIDTH = DEGREE_90;
bool line_following = true;
bool fire_searching = true;
bool fire_detected = false;
bool stop_aim = false;
bool fire_on_aim = false;
uint16_t adc_value[6];

// data enumerasi untuk lokasi api pada sensor
typedef enum {
    SENSOR_MID = 0,
    SENSOR_BOT,
    SENSOR_TOP
} Sensor;

Sensor fire_at = SENSOR_MID; // deklarasi, nilai awal

// inisiasi pin yang digunakan, input, output
void init_pin(void) {
	DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6);
	DDRC |= (1 << PC3) | (1 << PC4) | (1 << PC5);
	DDRC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2));
	DDRB |= (1 << PB1) | (1 << PB2) | (1 << PB3);
}

// inisiasi pin c sebagai adc
void init_adc(void) {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);
}

// inisiasi mode pwm
void init_pwm(void) {
	TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); // Fast pwm mode, non-inverting pin OCR0A (D6)
	TCCR0B |= (1 << CS02); // prescaler frekuensi clock x256 pin OCR0A (D6)
	
	TCCR1A |= (1 << COM1A1) | (1 << WGM10); // Fast pwm mode, non-inverting pin OCR1A (B1)
	TCCR1B |= (1 << CS12) | (1 << WGM12); // prescaler frekuensi clock x256 pin OCR1A (B1)
	
	OCR0A = PULSE_WIDTH;
	
	OCR1A = PULSE_WIDTH;
}

// mengonversi input adc (pin c)
uint16_t adc_read(uint8_t adc_channel) {
	ADMUX = (ADMUX & 0xF8) | (adc_channel & 0x07);
	ADCSRA |= (1 <<ADSC);
	
	while (ADCSRA & (1 << ADSC));
    
    return ADC;
}

// membaca pin-pin adc
void adc_check(void) {
	for (int channel = 0; channel < 7; channel++) {
		adc_value[channel] = adc_read(channel);
	}
}

// Kontrol motor line follower
void motor_forward(void) {
    PORTB |= (1 << PB2);
    PORTB |= (1 << PB3);
}

void motor_left(void) {
    PORTB &= ~(1 << PB2);
    PORTB |= (1 << PB3);
}

void motor_right(void) {
    PORTB |= (1 << PB2);
    PORTB &= ~(1 << PB3);
}

void motor_stop(void) {
    PORTB &= ~(1 << PB2);
    PORTB &= ~(1 << PB3);
}

// cek keberadaan api pada sensor
void fire_check(void) {
	if (adc_value[3] > FIRE_THRESHOLD) { // sensor tengah
		fire_detected = true;
		fire_at = SENSOR_MID;
	} else if (adc_value[4] > FIRE_THRESHOLD) { // sensor bawah
		fire_detected = true;
		fire_at = SENSOR_BOT;
	} else if (adc_value[5] > FIRE_THRESHOLD) { // sensor atas
		fire_detected = true;
		fire_at = SENSOR_TOP;
	}
}

// merotasikan servo 1 (horizontal) dari sudut rendah ke tinggi (kiri)
void aim_left(int PULSE_WIDTH_NOW, int PULSE_WIDTH_RESULT) {
	// loop sampai servo berada pada sudut maksimum atau api terdeteksi sensor
	for (int i = PULSE_WIDTH_NOW; i < PULSE_WIDTH_RESULT && !fire_detected; i += 1) {
		
		OCR0A = i;
		adc_check();
		_delay_ms(30);
		PULSE_WIDTH = i;
		fire_check();
	}
}

// merotasikan servo 1 (horizontal) dari sudut tinggi ke rendah (kanan)
void aim_right(int PULSE_WIDTH_NOW, int PULSE_WIDTH_RESULT) {
	// loop sampai servo berada pada sudut minimum atau api terdeteksi sensor
	for (int i = PULSE_WIDTH_NOW; i > PULSE_WIDTH_RESULT && !fire_detected; i -= 1) {
		
		OCR0A = i;
		adc_check();
		_delay_ms(30);
		PULSE_WIDTH = i;
		fire_check();
	}
}

// 
int main(void) {
	init_pin();
	init_pwm();
	init_adc();
	sei();
	
	while (1) {
		
		// reset variabel
		OCR1A = DEGREE_90;
		line_following = true;
		fire_searching = true;
		fire_on_aim = false;
		stop_aim = false;
		
		// reset indikator
		PORTD &= ~(1 << PD0);
		PORTD &= ~(1 << PD1);
		PORTD &= ~(1 << PD2);
		PORTD &= ~(1 << PD4);
		
		// loop line follower
		while (line_following) { // bisa diberi kondisi tambahan untuk memulai pemadam kebakaran selain saat motor stop
			
			adc_check();
			
			if (adc_value[1] > LINE_THRESHOLD) {
				motor_forward();
			} else if (adc_value[0] > LINE_THRESHOLD) {
				motor_left();
			} else if (adc_value[2] > LINE_THRESHOLD) {
				motor_right();
			} else {
				motor_stop();
				line_following = false;
			}
			
			_delay_ms(50);
		}
		
		// loop servo 1 (horizontal), memindai api
		while (fire_searching) {
			
			PORTD |= (1 << PD0);
			adc_check();
			
			aim_left(PULSE_WIDTH, DEGREE_180);
			
			if (fire_detected) {
				stop_aim = true; // agar posisi servo horizontal tetap, tidak rotasi ke kanan
				fire_searching = false;
			}
			
			if (!stop_aim) { // jika tidak terdeteksi api saat rotasi ke kiri, lanjut rotasi
				aim_right(PULSE_WIDTH, DEGREE_0);
				if (fire_detected) {
					fire_searching = false;
				}
			}
		}
		
		PORTD &= ~(1 << PD0);
		
		// loop servo 2 (vertikal), membidik dan memadamkan api
		while (fire_detected) {
			
			// jika api terdeteksi di sensor bawah
			if (fire_at == SENSOR_BOT) {
				PORTD |= (1 << PD2);
				
				// servo 2 rotasi kebawah sampai api terdeteksi di sensor tengah
				for (int i = DEGREE_90; i > DEGREE_0 && !fire_on_aim; i -= 1) {
					
					OCR1A = i;
					adc_check();
					_delay_ms(30);
					
					if (adc_value[3] > FIRE_THRESHOLD) {
						fire_on_aim = true;
						fire_at = SENSOR_MID;
					}
				}
				
				// jika api terdeteksi di sensor bawah
			} else if (fire_at == SENSOR_TOP) {
				PORTD |= (1 << PD1);
				
				// servo 2 rotasi keatas sampai api terdeteksi di sensor tengah
				for (int i = DEGREE_90; i < DEGREE_180 && !fire_on_aim; i += 1) {
					
					OCR1A = i;
					adc_check();
					_delay_ms(30);
					
					if (adc_value[3] > FIRE_THRESHOLD) {
						fire_on_aim = true;
						fire_at = SENSOR_MID;
					}
				}
			}
			
			// api terdeteksi di sensor tengah
			if (fire_at == SENSOR_MID) { 
				
				PORTD |= (1 << PD4); // water pump on
				
				// loop, selesai sat api tidak terdeteksi lagi di sensor tengah
				while (fire_on_aim) {
					
					_delay_ms(100);
					adc_check();
					if (adc_value[3] < FIRE_THRESHOLD) {
						fire_on_aim = false;
					}
					
				}
				
				PORTD &= ~(1 << PD4);	// water pump off
				fire_detected = false;
			}
		}
	}
}

