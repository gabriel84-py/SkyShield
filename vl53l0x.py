from machine import Pin, I2C
import time

class VL53L0X:
    """Classe pr capteur distance laser ToF VL53L0X"""
    
    # registres VL53L0X
    REG_IDENTIFICATION_MODEL_ID = 0xC0
    REG_SYSRANGE_START = 0x00
    REG_RESULT_RANGE_STATUS = 0x14
    
    def __init__(self, i2c_id=0, scl=17, sda=16, addr=0x29):
        """
        Init capteur VL53L0X
        
        Params :
        - i2c_id : num bus I2C (0 ou 1)
        - scl : pin horloge I2C
        - sda : pin data I2C
        - addr : adresse I2C (0x29 défaut)
        """
        # création bus I2C
        self.i2c = I2C(
            i2c_id,
            scl=Pin(scl),
            sda=Pin(sda),
            freq=400_000  # 400kHz (VL53L0X supporte)
        )
        
        self.addr = addr
        
        # vérifie détection capteur
        devices = self.i2c.scan()
        if self.addr not in devices:
            raise Exception(f"VL53L0X pas détecté à 0x{self.addr:02X}")
        
        # init capteur
        self._init_sensor()
        
        print(f"VL53L0X détecté à 0x{self.addr:02X}")
    
    def _init_sensor(self):
        """Init basique capteur (config défaut)"""
        try:
            # vérif model ID (doit être 0xEE)
            model_id = self.i2c.readfrom_mem(self.addr, self.REG_IDENTIFICATION_MODEL_ID, 1)[0]
            if model_id != 0xEE:
                print(f"Warning: Model ID = 0x{model_id:02X} (attendu 0xEE)")
            
            time.sleep(0.01)
            
        except Exception as e:
            print(f"Erreur init VL53L0X: {e}")
            raise
    
    def read_range_single(self):
        """
        Mesure distance en mode single shot
        
        Ret : distance en mm, ou None si erreur
        """
        try:
            # démarre mesure
            self.i2c.writeto_mem(self.addr, self.REG_SYSRANGE_START, b'\x01')
            
            # attends fin mesure (polling, max 100ms)
            timeout = 100  # ms
            start = time.ticks_ms()
            
            while True:
                # lit status
                status = self.i2c.readfrom_mem(self.addr, self.REG_RESULT_RANGE_STATUS, 1)[0]
                
                # bit 0 = data ready
                if status & 0x01:
                    break
                
                # timeout
                if time.ticks_diff(time.ticks_ms(), start) > timeout:
                    return None
                
                time.sleep_ms(1)
            
            # lit distance (2 octets à offset +10 du status register)
            data = self.i2c.readfrom_mem(self.addr, self.REG_RESULT_RANGE_STATUS + 10, 2)
            distance_mm = (data[0] << 8) | data[1]
            
            return distance_mm
            
        except Exception as e:
            print(f"Erreur lecture: {e}")
            return None
    
    def distance_mm(self):
        """Ret distance en mm"""
        return self.read_range_single()
    
    def distance_cm(self):
        """Ret distance en cm"""
        mm = self.read_range_single()
        return mm / 10.0 if mm is not None else None
    
    def distance_m(self):
        """Ret distance en m"""
        mm = self.read_range_single()
        return mm / 1000.0 if mm is not None else None
    
    def distance_avg(self, samples=5):
        """
        Mesure av moyenne sur N échantillons (réduit bruit)
        
        Params : samples = nb mesures (défaut 5)
        Ret : distance en m, ou None si erreur
        """
        measures = []
        
        for _ in range(samples):
            dist = self.distance_m()
            if dist is not None and dist < 4.0:  # filtre valeurs aberrantes
                measures.append(dist)
            time.sleep_ms(10)  # 10ms entre mesures
        
        if len(measures) == 0:
            return None
        
        # ret médiane (plus robuste que moyenne)
        measures.sort()
        mid = len(measures) // 2
        return measures[mid]
    
    def is_in_range(self, distance_m):
        """Vérifie si distance valide (0.03-2m pr VL53L0X)"""
        return 0.03 <= distance_m <= 2.0 if distance_m else False