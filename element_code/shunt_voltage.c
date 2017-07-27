	uint32_t shunt_voltage;
	INA219_GetShuntVoltageMv(INA219_ADDR, &shunt_voltage);
	return shunt_voltage;