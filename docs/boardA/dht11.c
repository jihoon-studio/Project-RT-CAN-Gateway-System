/*
 * dht11.c
 *
 *  Created on: 2026. 1. 15.
 *      Author: User
 */

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dht11.h"
#include "cmsis_os.h"   // osDelay
#include "FreeRTOS.h"   // taskENTER_CRITICAL
#include "task.h"

extern void delay_us(unsigned long _us);
void dht11_dataline_input(void);
void dht11_dataline_output(void);
int wait_readpin(GPIO_PinState _pinstate, uint32_t _timeout);

int dht11_read_data(uint8_t *temp, uint8_t *humi) {
    uint8_t data[5] = {0};
    uint32_t us_counter = 0;

    // [1] 시작 신호 보내기 (18ms 대기)
    // 이 시간 동안은 osDelay를 써서 다른 태스크(CAN, LED 등)가 동작하도록 양보함
    dht11_dataline_output();
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
    osDelay(20); // 20ms 대기 (RTOS Delay)

    // [2] 데이터 읽기 시작 (타이밍 중요 구간)
    // 이 구간에서는 절대 방해받으면 안 되므로 인터럽트 차단
    taskENTER_CRITICAL();

    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
    delay_us(30);            // 30us 대기 (Microsecond Delay)
    dht11_dataline_input();  // 입력 모드로 전환

    // 센서 응답 확인 (Low 80us -> High 80us)
    if (wait_readpin(GPIO_PIN_RESET, 100) < 0) {
        taskEXIT_CRITICAL(); // 실패 시 반드시 락 해제
        return -1;
    }
    if (wait_readpin(GPIO_PIN_SET, 100) < 0) {
        taskEXIT_CRITICAL();
        return -2;
    }

    // 데이터 전송 시작 대기
    while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_SET);

    // 40비트 데이터 읽기
    for (int i = 0; i < 5; i++) {
        for (int j = 7; j >= 0; j--) {
            // Low 구간 대기 (50us)
            while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_RESET);

            // High 구간 길이 측정 (26~28us = '0', 70us = '1')
            us_counter = 0;
            while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_SET) {
                delay_us(1);
                us_counter++;
                if (us_counter > 100) {
                    taskEXIT_CRITICAL(); // 타임아웃
                    return -3;
                }
            }

            // 40us보다 길면 '1'로 인식
            if (us_counter > 40) {
                data[i] |= (1 << j);
            }
        }
    }

    // [3] 읽기 완료, 락 해제
    taskEXIT_CRITICAL();

    // 체크섬 검증
    if (data[4] == (data[0] + data[1] + data[2] + data[3])) {
        *humi = data[0]; // 습도 정수부
        *temp = data[2]; // 온도 정수부
        return 0; // 성공
    } else {
        return -4; // 체크섬 에러
    }
}

void dht11_dataline_input(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = DHT11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

void dht11_dataline_output(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = DHT11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

int wait_readpin(GPIO_PinState _pinstate, uint32_t _timeout) {
	uint32_t us_counter = 0;
	while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == _pinstate) {
		delay_us(2);
		us_counter += 2;
		if (us_counter > _timeout) {
			return -1;
		}
	}
	return 0;
}

void dht11_main(void) {
	enum state_t {
		OK, TIMEOUT, VALUE_ERROR, TRANS_ERROR
	};
	enum state_t state = OK;
	uint32_t us_counter = 0;

	int data[6] = { 0 };

	while (1) {
		for (int i = 0; i < 6; i++)
			data[i] = 0;

		state = OK;

		// ========== step1 request start signal : STM32 ==========
		dht11_dataline_output();
		HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
		HAL_Delay(100);		// 100 ms

		HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
		HAL_Delay(20);		// 20 ms

		HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
		delay_us(30);

		dht11_dataline_input();

		if (wait_readpin(1, 50) < 0) {
			state = TIMEOUT;
		}

		// ========== step2 response start signal from DHT11 ==========
		if (state == OK) {
			if (wait_readpin(0, 100) < 0) {
				state = TIMEOUT;
			}
		}

		if (state == OK) {
			if (wait_readpin(1, 100) < 0) {
				state = TIMEOUT;
			}
		}

		// ==========  ==========
		if (state == OK) {
			int pulse[8] = { 0, };

			for (int i = 0; i < 5; i++) {
				for (int j = 7; j >= 0; j--) {
					if (wait_readpin(0, 70) < 0) {
						state = TIMEOUT;
						i = 5;
						j = -1;
					}

					if (state == OK) {
						if (wait_readpin(1, 90) < 0) {
							state = TIMEOUT;
							i = 5;
							j = -1;
						}

						if (state == OK) {
							if (us_counter < 40)
								pulse[j] = 0;
							else if (us_counter > 40)
								pulse[j] = 1;
						}
					}
				}

				if (state == OK) {
					data[i] = pulse[0] << 0 | pulse[1] << 1 | pulse[2] << 2
							| pulse[3] << 3 | pulse[4] << 4 | pulse[5] << 5
							| pulse[6] << 6 | pulse[7] << 7;
				}
			}

			if (state == OK) {
				if (data[4] != data[0] + data[1] + data[2] + data[3]) {
					state = VALUE_ERROR;
				}
			}

			delay_us(60);
			us_counter = 0;
			while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == 0) {
				delay_us(2);
				us_counter += 2;
				if (us_counter > 90) {
					break;
				}
			}
		}

		if (state == OK) {
			printf("[TEMP]: %d.%dC\n", data[2], data[3]);
			printf("[HUMI]: %d.%d%%\n", data[0], data[1]);
		} else //if (state != OK)
		{
			printf("error code: %d\n", state);
			printf("data: 0x%02x 0x%02x 0x%02x 0x%02x\n", data[0], data[1],
					data[2], data[3]);
		}

		HAL_Delay(2000);
	}
}
