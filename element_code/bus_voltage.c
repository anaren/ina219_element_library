	uint32_t bus_voltage;
	INA219_GetBusVoltageMv(INA219_ADDR, &bus_voltage);
	return bus_voltage;