	int32_t current_ua;
	INA219_GetCurrent(INA219_ADDR, &current_ua);
	return current_ua;