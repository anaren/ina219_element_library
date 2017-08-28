	int32_t bus_voltage;
	INA219_GetBusVoltage(INA219_ADDR, &bus_voltage);
	return bus_voltage;