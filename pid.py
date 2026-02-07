import time

class PID:
    """Contrôleur PID av anti-windup et limites"""
    
    def __init__(self, kp, ki, kd, output_limits=(-100, 100), integral_limit=50):
        """
        Init PID
        
        Params :
        - kp, ki, kd : gains PID
        - output_limits : (min, max) sortie correction
        - integral_limit : limite accumulation I (anti-windup)
        """
        # gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # limites
        self.min_out, self.max_out = output_limits
        self.integral_limit = integral_limit
        
        # états internes
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.ticks_us()
        
        # debug (optionnel)
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
    
    def compute(self, setpoint, measurement):
        """
        Calc correction PID

        vivelesmathsquememeenprepailsfontpas....
        j'ai dus regarder 40 videos d'indiens pour juste cette méthode...
        
        Params :
        - setpoint : cible (ex: roll = 0°)
        - measurement : mesure actuelle (ex: roll = -5°)
        
        Ret : correction à appliquer
        """
        # calc temps écoulé
        now = time.ticks_us()
        dt = time.ticks_diff(now, self.last_time) / 1_000_000  # sec
        self.last_time = now
        
        # évite division par 0
        if dt <= 0 or dt > 1.0:  # + de 1 sec = anormal
            dt = 0.01
        
        # calc erreur
        error = setpoint - measurement
        
        # terme P (proportionnel)
        self.p_term = self.kp * error
        
        # terme I (intégral av limite)
        self.integral += error * dt
        
        # anti-windup : limite intégrale
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
        
        self.i_term = self.ki * self.integral
        
        # terme D (dérivé)
        derivative = (error - self.last_error) / dt
        self.d_term = self.kd * derivative
        
        # somme PID
        output = self.p_term + self.i_term + self.d_term
        
        # limite sortie
        if output > self.max_out:
            output = self.max_out
        elif output < self.min_out:
            output = self.min_out
        
        # mémo pr prochain cycle
        self.last_error = error
        
        return output
    
    def reset(self):
        """Reset états internes (au démarrage !)"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.ticks_us()
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
    
    def set_gains(self, kp=None, ki=None, kd=None):
        """Change gains en direct (pr tuning live)"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
    
    def get_terms(self):
        """Ret termes PID séparés (debug)"""
        return self.p_term, self.i_term, self.d_term