	int32_t power;
	INA219_GetPower(INA219_ADDR, &power);
	return power;