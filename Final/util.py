import statistics
"""
Smoothes a value
"""
class ValueSmoother:
    def __init__(self, n, init_value) -> None:
        self.values = [init_value] * n
    
    def update_value(self, value):
        self.values.append(value)
        self.values.pop(0)
        
        return self.get_current_reading()
    
    def get_current_reading(self):
        return sum(self.values) / len(self.values)
    
"""
Majority voting
"""
class MajorityVoter:
    def __init__(self, n, init_value) -> None:
        self.values = [init_value] * n
        self.clear(init_value)
    
    def update_value(self, value):
        self.values.append(value)
        self.values.pop(0)
        
        return self.get_current_reading()
    
    def get_current_reading(self):
        return statistics.mode(self.values)
    
    def clear(self, value):
        for i in range(0, len(self.values)):
            self.values[i] = value