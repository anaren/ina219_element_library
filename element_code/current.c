	uint32_t current_ma;
	INA219_GetCurrentMa(INA219_ADDR, &current_ma);
	return current_ma;