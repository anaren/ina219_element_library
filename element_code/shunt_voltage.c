	float shunt_voltage;
	INA219_GetShuntVoltageMv(INA219_SLAVE_BASE_ADDR, &shunt_voltage);
	return shunt_voltage;