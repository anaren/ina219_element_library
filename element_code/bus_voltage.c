	float bus_voltage;
	INA219_GetBusVoltageV(INA219_SLAVE_BASE_ADDR, &bus_voltage);
	return bus_voltage;