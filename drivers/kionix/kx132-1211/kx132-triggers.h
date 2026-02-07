#ifndef _KX132_TRIGGERS_H
#define _KX132_TRIGGERS_H



#define MINIMUM_EXPECTED_GPIO_PORT_NAME_LENGTH (3U)

int kx132_reinitialize_interrupt_port(const struct device *dev, uint32_t option);



#endif // _KX132_TRIGGERS_H
