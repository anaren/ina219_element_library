	int32_t shunt_voltage;
	INA219_GetShuntVoltage(INA219_ADDR, &shunt_voltage);
	return shunt_voltage;