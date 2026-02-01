from machine import SPI, Pin
import os
import time

class SDLogger:
    """Classe pr logger data drone sur carte SD"""
    
    def __init__(self, cs_pin=17, sck_pin=18, mosi_pin=19, miso_pin=16, spi_id=0):
        """
        Init module SD via SPI
        
        Params :
        - cs_pin: Chip Select (def 17)
        - sck_pin: Clock (def 18)
        - mosi_pin: Master Out Slave In (def 19)
        - miso_pin: Master In Slave Out (def 16)
        - spi_id: bus SPI (0 ou 1, def 0)
        """
        # init SPI (vitesse basse pr compatibilité max)
        self.spi = SPI(
            spi_id,
            baudrate=100_000,  # 100 kHz pr stabilité (au lieu de 1 MHz)
            polarity=0,
            phase=0,
            sck=Pin(sck_pin),
            mosi=Pin(mosi_pin),
            miso=Pin(miso_pin)
        )
        
        # init pin CS
        self.cs = Pin(cs_pin, Pin.OUT)
        self.cs.value(1)  # désactive SD (actif bas)
        time.sleep(0.1)   # attente stabilisation
        
        # monte filesystem SD av retry
        self.mounted = False
        for tentative in range(3):
            try:
                import sdcard
                self.sd = sdcard.SDCard(self.spi, self.cs)
                time.sleep(0.1)  # pause avant montage
                os.mount(self.sd, '/sd')
                print(f"SD montée sur /sd (tentative {tentative + 1})")
                self.mounted = True
                break
            except Exception as e:
                print(f"Tentative {tentative + 1}/3 échouée: {e}")
                time.sleep(0.5)
        
        if not self.mounted:
            print("Impossible de monter SD après 3 tentatives")
        
        # nom fichier log
        self.log_file = '/sd/vol_log.csv'
        
        # crée fichier av header si existe pas
        if self.mounted:
            self._create_log_file()
    
    def _create_log_file(self):
        """Crée fichier log av header CSV si existe pas"""
        try:
            # vérifie si fichier existe déjà
            try:
                with open(self.log_file, 'r') as f:
                    pass  # fichier existe
                print(f"Fichier log existe: {self.log_file}")
            except:
                # crée new fichier av header
                with open(self.log_file, 'w') as f:
                    f.write("timestamp_ms,roll,pitch,yaw\n")
                print(f"Nouveau fichier log: {self.log_file}")
        except Exception as e:
            print(f"Erreur création fichier: {e}")
    
    def log(self, roll, pitch, yaw):
        """
        Enregistre angles ds fichier CSV
        
        Params : roll, pitch, yaw (en degrés)
        Ret : True si succès, False sinon
        """
        if not self.mounted:
            return False
        
        try:
            # timestamp en ms dp démarrage
            timestamp = time.ticks_ms()
            
            # écrit ligne ds fichier
            with open(self.log_file, 'a') as f:
                f.write(f"{timestamp},{roll:.2f},{pitch:.2f},{yaw:.2f}\n")
            
            return True
        except Exception as e:
            print(f"Erreur écriture log: {e}")
            return False
    
    def new_flight(self):
        """Crée new fichier pr nouveau vol (timestampé)"""
        if not self.mounted:
            return False
        
        try:
            # timestamp pr nom fichier unique
            timestamp = time.ticks_ms()
            self.log_file = f'/sd/vol_{timestamp}.csv'
            
            # crée fichier av header
            with open(self.log_file, 'w') as f:
                f.write("timestamp_ms,roll,pitch,yaw\n")
            
            print(f"Nouveau vol: {self.log_file}")
            return True
        except Exception as e:
            print(f"Erreur création vol: {e}")
            return False
    
    def get_stats(self):
        """Ret nb lignes loggées ds fichier actuel"""
        if not self.mounted:
            return 0
        
        try:
            with open(self.log_file, 'r') as f:
                lines = sum(1 for _ in f) - 1  # -1 pr header
            return lines
        except:
            return 0
    
    def unmount(self):
        """Démonte SD proprement"""
        if self.mounted:
            try:
                os.umount('/sd')
                print("SD démontée")
                self.mounted = False
            except Exception as e:
                print(f"Erreur démontage: {e}")