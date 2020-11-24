import math

from bmp_280 import BMP280

bmp = BMP280(port=0, mode=BMP280.NORMAL_MODE, oversampling_p=BMP280.OVERSAMPLING_P_x16,
             oversampling_t=BMP280.OVERSAMPLING_T_x2,
             filter=BMP280.IIR_FILTER_x16, standby=BMP280.T_STANDBY_0p5)


def getAltAndTemp():
    temp = round(bmp.read_temperature(), 1)
    pressure = round(bmp.read_pressure(), 4)

    return round(-((math.log(pressure * 100 / 101325) * 287.053) * (temp + 459.67) * (5 / 9) / 9.8), 1), temp
