#include "TSSP_Array.h"

void TSSP_Array::init() {
    for(uint8_t i = 0; i < TSSP_NUM; i++) {
        pinMode(pins[i], INPUT);
        const float angle_rad = DEG_TO_RAD * (i * 30.0f);
        x_values[i] = cosf(angle_rad);
        y_values[i] = sinf(angle_rad);
	}
}

void TSSP_Array::update() {
	for(uint8_t i = 0; i < READ_NUM; i++) {
		for(uint8_t j = 0; j < TSSP_NUM; j++) {
		    values[j] += 1 - digitalRead(pins[j]);
	    }
	}
    uint16_t total_value = 0;
    uint8_t max_value = 0;
    uint8_t second_max_value = 0;
    uint8_t max_index = 0;
    float raw_x = 0.0f;
    float raw_y = 0.0f;

    for(uint8_t i = 0; i < TSSP_NUM; i++) {
        total_value += values[i];

        if(values[i] > max_value) {
            second_max_value = max_value;
            max_value = values[i];
            max_index = i;
        } else if(values[i] > second_max_value) {
            second_max_value = values[i];
        }

        if(values[i] > noise_floor) {
            const float weight = (float)(values[i] - noise_floor);
            raw_x += x_values[i] * weight;
            raw_y += y_values[i] * weight;
        }
	}

    direction_simple = (max_value <= noise_floor) ? 0 : max_index + 1;

    // Filter the vector in cartesian space to avoid angle wrap-around jitter.
    filtered_x += direction_alpha * (raw_x - filtered_x);
    filtered_y += direction_alpha * (raw_y - filtered_y);

    if(max_value <= noise_floor || (fabsf(filtered_x) < 0.001f && fabsf(filtered_y) < 0.001f)) {
        direction_advanced = 0;
    } else {
        float angle_deg = atan2f(filtered_y, filtered_x) * RAD_TO_DEG;
        angle_deg += angle_offset_deg;
        while(angle_deg < 0.0f) {
            angle_deg += 360.0f;
        }
        while(angle_deg >= 360.0f) {
            angle_deg -= 360.0f;
        }
        direction_advanced = (uint16_t)roundf(angle_deg);
    }

    const float baseline = (float)total_value / (float)TSSP_NUM;
    const float dominant = max_value > baseline ? (float)max_value - baseline : 0.0f;
    const float support = second_max_value > baseline ? (float)second_max_value - baseline : 0.0f;
    const float strength_input = (max_value <= noise_floor) ? 0.0f : (dominant * 0.75f + support * 0.25f);

    filtered_strength += strength_alpha * (strength_input - filtered_strength);
    const float strength_percent = constrain((filtered_strength * 100.0f) / (float)READ_NUM, 0.0f, 100.0f);
    strength = (uint8_t)roundf(strength_percent);

	for(uint8_t i = 0; i < TSSP_NUM; i++) {
		values[i] = 0;
		sorted_values[i] = 0;
		indexes[i] = 0;
	}
}


uint16_t TSSP_Array::get_direction_advanced() {
    return direction_advanced;
}

uint8_t TSSP_Array::get_direction_simple() {
    return direction_simple;
}

uint8_t TSSP_Array::get_strength() {
    return strength;
}