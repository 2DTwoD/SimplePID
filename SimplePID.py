import TrendViewer

# дескриптор для изменения коэффициентов регулятора
class KoefChangeDescriptor:

    def __init__(self, *filt):
        self.filt = filt

    def __set_name__(self, objtype, name):
        self.name = "_SimplePID__"+name

    def __get__(self, obj, objtype):
        return getattr(obj, self.name)

    def __set__(self, obj, value):
        #Постоянная часть сообщения для действия
        pre_txt = f"Object {obj.name}: "
        text_name = self.name[12:]
        #Проверка на разрешенное значение переменной
        if getattr(obj, self.name) == value:
            print(pre_txt + f"Parameter {text_name} is already {value}")
            return
        check = False
        if self.filt[1] == "number":
            check = type(value) == int or type(value) == float
        elif self.filt[1] == "text":
            check = type(value) == str
        if not check:
            print(pre_txt + f"Parameter {text_name} must be {self.filt[1]}")
            return
        #Ограничения назначения переменной
        if self.filt[0] == "free":
            result = value
        elif self.filt[0] == "limit":
            up_lim = self.check_value(obj, self.filt[3])
            down_lim = self.check_value(obj, self.filt[2])
            result = self.limiter(pre_txt, text_name, value, up_lim, down_lim)
        elif self.filt[0] == "choise":
            if value in self.filt[2:]:
                result = value
            else:
                print(pre_txt + f"Parameter {text_name} must be in {self.filt[2:]}")
                result = getattr(obj, self.name)
        #Назначение переменной
        setattr(obj, self.name, result)
        #Обновление регулятора
        if text_name == "mode":
            obj.update_PID()
        #Перерасчет коэффициентов для рекурентного метода
        if obj.mode == "Recurrent" and text_name in ("Kp", "Ti", "Td", "T"):
            setattr(obj, "_SimplePID__rec_upKoef", True)
        #Сообщение о произведенном действии
        print(pre_txt + f"Parameter {text_name} now is {result}")
    # функция для проверки необходимости ограничения параметра
    def check_value(self, obj, value):
        if type(value) == str:
            if value == "free":
                return None
            return getattr(obj, value)
        else:
            return value
    # функция ограничения изменяемого параметра
    def limiter(self, pre_txt, text_name, value, up_lim, down_lim):
        if up_lim is not None and value > up_lim:
            print(pre_txt + f"Parameter {text_name} must be lower that {up_lim}")
            return up_lim
        elif down_lim is not None and value < down_lim:
            print(pre_txt + f"Parameter {text_name} must be higher that {down_lim}")
            return down_lim
        else:
            return value

# ПИД регулятор, реализованный тремя разными способами:
# - чаще всего применяемый на практике в ПЛК
# - классический для САУ
# - рекурентный
class SimplePID:
    # коэффициенты регулятора, как экземпляры дескриптора (для удобства изменения параметров)
    mode = KoefChangeDescriptor("choise", "text", "PLC", "SAU", "Recurrent")
    SP = KoefChangeDescriptor("free", "number")
    Kp = KoefChangeDescriptor("limit", "number", 0, "free")
    Ti = KoefChangeDescriptor("limit", "number", 0, "free")
    Td = KoefChangeDescriptor("limit", "number", 0, "free")
    T = KoefChangeDescriptor("limit", "number", 0.001, "free")
    up_lim = KoefChangeDescriptor("limit", "number", "down_lim", "free")
    down_lim = KoefChangeDescriptor("limit", "number", "free", "up_lim")
    dead_band = KoefChangeDescriptor("limit", "number", 0, "free")
    dir = KoefChangeDescriptor("choise", "number", -1, 1)

    # конструктор с начальными параметрами
    def __init__(self, name,  **kwargs):
        self.name = name
        self.__SAU_choise = True
        self.__mode = "SAU"
        self.__SP = 0
        self.__OUT = 0
        self.__delta = 0
        self.__prev_delta = 0
        self.__sum_delta = 0
        self.__dif_delta = 0
        self.__T = 1
        self.__Kp = 1
        self.__Ti = 0
        self.__Td = 0
        self.__dead_band = 0
        self.__up_lim = 100
        self.__down_lim = 0
        self.__dir = 1
        self.__rec_delta = 0
        self.__rec_K1 = 0
        self.__rec_K2 = 0
        self.__rec_K3 = 0
        self.__rec_upKoef = False
        self.set_params(**kwargs)
        self.update_PID()

    # метод возвращает кортеж всех параметров регулятора
    def get_all_params(self):
        return ("mode", "SP", "Kp", "Ti", "Td", "T", "up_lim", "down_lim", "dead_band", "dir")

    # метод для обновления параметров регулятора, используется, если был изменен параметр mode
    def update_PID(self):
        self.__delta = 0
        self.__prev_delta = 0
        self.__sum_delta = 0
        self.__dif_delta = 0
        self.__rec_pprev_delta = 0
        if self.__mode == "SAU":
            self.__PID_func = self.__SAUPLC_PID
            self.__SAU_choise = True
        elif self.__mode == "PLC":
            self.__PID_func = self.__SAUPLC_PID
            self.__SAU_choise = False
        elif self.__mode == "Recurrent":
            self.__PID_func = self.__Recurrent_PID

    # метод для задания одного или нескольких параметров
    def set_params(self, **params):
        for key, value in params.items():
            try:
                getattr(self, key)
            except AttributeError:
                print(f"In object {self.name}: parameter {key} not exist")
            else:
                setattr(self, key, value)

    # метод для расчета выхода регулятора
    def lets_OUT(self, PV):
        delta = self.__dir * (self.__SP - PV)
        return self.__PID_func(delta)

    # метод с алгоритмом ПИД регулятора в режиме САУ или ПЛК
    def __SAUPLC_PID(self, delta):
        if abs(delta) > self.__dead_band:
            self.__delta = delta
            self.__sum_delta += delta
            self.__dif_delta = delta - self.__prev_delta
            self.__prev_delta = delta
        P_part = self.__delta
        if self.__Ti != 0:
            I_part = self.__T / self.__Ti * self.__sum_delta
        else:
            I_part = 0
        D_part = self.__Td / self.__T * self.__dif_delta
        if self.__SAU_choise:
            self.__OUT = self.__Kp * P_part + I_part + D_part
        else:
            self.__OUT = self.__Kp * (P_part + I_part + D_part)
        return self.limiter(self.__OUT, self.__up_lim, self.__down_lim)

    # метод с алгоритмом ПИД регулятора в рекурентном режиме
    def __Recurrent_PID(self, delta):
        if self.__rec_upKoef:
            self.__new_rec_koef()
        if abs(delta) > self.__dead_band:
            self.__rec_pprev_delta = self.__prev_delta
            self.__prev_delta = self.__delta
            self.__delta = delta
            part1 = self.__delta * self.__rec_K1
            part2 = self.__prev_delta * self.__rec_K2
            part3 = self.__rec_pprev_delta * self.__rec_K3
            self.__OUT += part1 + part2 + part3
        return self.limiter(self.__OUT, self.__up_lim, self.__down_lim)

    # метод для расчета составляющих коэффициентов для рекурентного режима регулятора
    def __new_rec_koef(self):
        if self.__Ti != 0:
            T_div_Ti = self.__T / self.__Ti
        else:
            T_div_Ti = 0
        Td_div_T = self.__Td / self.__T
        self.__rec_K1 = self.__Kp + T_div_Ti + Td_div_T
        self.__rec_K2 = -self.__Kp - 2 * Td_div_T
        self.__rec_K3 = Td_div_T
        self.__rec_upKoef = False

    # метод для ограничения значения в зависимости от пределов
    def limiter(self, value, up_lim, down_lim):
        if value > up_lim:
            return up_lim
        elif value < down_lim:
            return down_lim
        else:
            return value

# функция для симуляции изменения переменной процесса
def new_data(pid):
    pv = 0
    pv_db = []
    for i in range(10000):
        pv_db.append(pv)
        out = pid.lets_OUT(pv)
        pv += out / 100
    return [[i for i in range(len(pv_db))], pv_db]

# пробные последовательности
if __name__ == "__main__":
    pid1 = SimplePID("pid1", mode="SAU", SP=50, Kp=0.3, Ti=300, Td=0, T=1, up_lim=100, down_lim=-100, dead_band=0, dir=1)
    pid2 = SimplePID("pid2", mode="PLC", SP=50, Kp=0.2, Ti=400, Td=0, T=1, up_lim=100, down_lim=-100, dead_band=0, dir=1)
    pid3 = SimplePID("pid3", mode="Recurrent", SP=50, Kp=0.1, Ti=100, Td=0, T=1, up_lim=100, down_lim=-100, dead_band=0, dir=1)
    #pid1.set_params(mode="Recurrent", SP=[], Kp="1", Ti=True, Td=0, T=0.001, up_lim=100, down_lim=-100, dead_band=0, dir=0)
    #pid1.set_params(mode="Recurrent", SP=30, Kp=0.5)
    #pid1.dir = 1
    #pid1.SP = 30
    #pid1.up_lim = {}
    pv_db = new_data(pid1)
    new_trend = TrendViewer.TrendViewer(pv_db[0], pv_db[1], geom_w=800, geom_h=600)
    pv_db = new_data(pid2)
    new_trend.add_trend(pv_db[0], pv_db[1])
    pv_db = new_data(pid3)
    new_trend.add_trend(pv_db[0], pv_db[1])
    new_trend.create_win()
