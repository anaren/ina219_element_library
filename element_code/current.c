	float current_ma;
	INA219_GetCurrentMa(INA219_SLAVE_BASE_ADDR, &current_ma);
	return current_ma;