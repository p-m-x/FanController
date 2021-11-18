# MODBUS registers

All registers starts from 0x00 address

| Name | Type | Offset | Default |
|--|--|--|--|
| Slave address | holding | 0 | 20 |
| Temperature threshold| holding | 1 | 30 |
| Temperature hysteresis| holding | 2 | 5 |
| Fan speed (percent 0-100) | holding | 3 |
| Error | holding | 4 |
| Temperature #1 | input | 0 
| Temperature #2 | input | 1